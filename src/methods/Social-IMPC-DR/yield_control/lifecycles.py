"""Lifecycle policies own per-drone trip state and termination logic.

Two peers are provided:

- ``OneWay``     drones land once, then are teleported off-pad
                 (legacy phase 1-4 behaviour).
- ``RoundTrip``  Source -> Pad -> Source FSM with unload timer; only
                 INBOUND drones compete for the pad.

A lifecycle never picks the winner itself — it filters who is eligible
and may force everyone to yield (e.g. while the pad is busy unloading).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Set

import numpy as np

import SET

from .context import Context, PAD_CENTER

INBOUND   = "INBOUND"
UNLOADING = "UNLOADING"
OUTBOUND  = "OUTBOUND"
DONE      = "DONE"


class Lifecycle:
    name: str = "lifecycle"

    def cleanup_landed(self, ctx: Context) -> None:
        pass

    def filter_inbound(self, active: List[int], ctx: Context) -> List[int]:
        return list(active)

    def force_yield(
        self, inbound: List[int], ctx: Context
    ) -> Optional[Dict]:
        return None

    def step_update(self, ctx: Context) -> None:
        pass

    def all_finished(self, ctx: Context) -> Optional[bool]:
        """Return True/False to override default termination, or None to
        defer to ``all(target_reached)``."""
        return None

    def bind(self, target_array) -> None:
        """Optional hook used only by lifecycles that swap goals."""
        pass


class OneWay(Lifecycle):
    """Drones land once and are teleported off-pad to clear the way."""

    name = "one_way"

    def cleanup_landed(self, ctx: Context) -> None:
        for j in range(ctx.num_moving_drones):
            if ctx.target_reached[j]:
                ctx.agent_list[j].p = np.array([100.0, 100.0])
                ctx.agent_list[j].v = np.zeros(2)
                ctx.agent_list[j].state = np.append(
                    ctx.agent_list[j].p, ctx.agent_list[j].v
                )
                ctx.agent_list[j].pre_traj = np.tile(
                    np.array([100.0, 100.0]), (ctx.K + 1, 1)
                )


class RoundTrip(Lifecycle):
    """Source -> Pad -> Source FSM with unload timer.

    Only INBOUND drones are eligible candidates each step; OUTBOUND
    drones still run MPC (so they fly home) but never claim the pad.
    While any drone is UNLOADING, all INBOUND candidates are forced to
    yield to avoid landing on an occupied pad.
    """

    name = "round_trip"

    def __init__(
        self,
        return_points=None,
        n_trips: int = 2,
        unload_steps: int = 5,
    ) -> None:
        self._return_points = [
            np.array(rp, dtype=float) for rp in (return_points or [])
        ]
        self._n_trips = max(1, int(n_trips))
        self._unload_steps = max(0, int(unload_steps))
        self._state: Dict[int, str] = {}
        self._unload_remaining: Dict[int, int] = {}
        self._trips_done: Dict[int, int] = {}
        self._target_ref = None
        self._initialized = False

    def bind(self, target_array) -> None:
        self._target_ref = target_array

    def _ensure_init(self, ctx: Context) -> None:
        if self._initialized:
            return
        for j in range(ctx.num_moving_drones):
            self._state[j] = INBOUND
            self._unload_remaining[j] = 0
            self._trips_done[j] = 0
        # Stamp each drone's home pad onto its uav for visualisation.
        for j in range(min(ctx.num_moving_drones, len(ctx.agent_list))):
            if j < len(self._return_points):
                ctx.agent_list[j].home_pad = self._return_points[j].copy()
        self._initialized = True

    def filter_inbound(self, active: List[int], ctx: Context) -> List[int]:
        self._ensure_init(ctx)
        return [j for j in active if self._state.get(j, INBOUND) == INBOUND]

    def force_yield(
        self, inbound: List[int], ctx: Context
    ) -> Optional[Dict]:
        if not self._initialized:
            return None
        # If the pad is occupied by an UNLOADING drone, no INBOUND drone
        # may approach -- everyone orbits until the pad clears. This
        # prevents MPC infeasibility from trying to land on an occupied pad.
        pad_busy = any(s == UNLOADING for s in self._state.values())
        if pad_busy:
            return {
                "allowed":  None,
                "yielding": set(inbound),
                "method":   "pad_busy",
                "scores":   {},
            }
        return None

    def cleanup_landed(self, ctx: Context) -> None:
        self._ensure_init(ctx)

        for j in range(ctx.num_moving_drones):
            if not ctx.target_reached[j]:
                continue
            state = self._state[j]

            if state == INBOUND:
                self._state[j] = UNLOADING
                self._unload_remaining[j] = self._unload_steps
                self._park(ctx.agent_list[j], ctx.agent_list[j].p, ctx.K)
                print(
                    f"  [round_trip] Drone {j}: INBOUND -> UNLOADING "
                    f"(trip {self._trips_done[j] + 1}/{self._n_trips})"
                )

            elif state == OUTBOUND:
                self._trips_done[j] += 1
                home = ctx.agent_list[j].home_pad
                if self._trips_done[j] >= self._n_trips:
                    self._state[j] = DONE
                    self._park(ctx.agent_list[j], home, ctx.K)
                    print(
                        f"  [round_trip] Drone {j}: OUTBOUND -> DONE "
                        f"(parked at home {home.tolist()}, "
                        f"completed {self._trips_done[j]} trips)"
                    )
                else:
                    self._state[j] = INBOUND
                    self._switch_target(j, ctx, PAD_CENTER)
                    print(
                        f"  [round_trip] Drone {j}: OUTBOUND -> INBOUND "
                        f"(starting trip {self._trips_done[j] + 1})"
                    )

            elif state == DONE:
                self._park(
                    ctx.agent_list[j], ctx.agent_list[j].home_pad, ctx.K
                )
            # UNLOADING is advanced by step_update.

    def step_update(self, ctx: Context) -> None:
        self._ensure_init(ctx)
        for j in range(ctx.num_moving_drones):
            if self._state[j] != UNLOADING:
                continue
            self._unload_remaining[j] -= 1
            if self._unload_remaining[j] <= 0:
                self._state[j] = OUTBOUND
                rp = (
                    self._return_points[j]
                    if j < len(self._return_points) else PAD_CENTER
                )
                self._switch_target(j, ctx, rp)
                print(
                    f"  [round_trip] Drone {j}: UNLOADING -> OUTBOUND "
                    f"(heading to {rp.tolist()})"
                )

    def all_finished(self, ctx: Context) -> Optional[bool]:
        if not self._initialized:
            return False
        return all(
            self._state.get(j) == DONE
            for j in range(ctx.num_moving_drones)
        )

    @staticmethod
    def _park(agent, position, K) -> None:
        agent.p = np.array(position, dtype=float)
        agent.v = np.zeros(2)
        agent.state = np.append(agent.p, agent.v)
        agent.pre_traj = np.tile(agent.p, (K + 1, 1))

    def _switch_target(self, j: int, ctx: Context, new_target) -> None:
        new_target = np.array(new_target, dtype=float)
        agent = ctx.agent_list[j]
        agent.change_target(new_target.copy())
        if self._target_ref is not None:
            self._target_ref[j] = new_target.copy()
        # IMPORTANT: run.run_one_agent re-asserts SET.target[j] every MPC
        # step, so we must update the SET-level target too -- otherwise the
        # change_target above is overwritten on the next step.
        if j < len(SET.target):
            SET.target[j] = new_target.copy()
        ctx.target_reached[j] = False
        # Warm-start MPC from the current position so the first solve does
        # not dart back toward the old goal.
        agent.pre_traj = np.tile(agent.p, (agent.K + 1, 1))
        agent.cost_index = agent.K
        # Clear MPC's "I've converged on the old target" termination flags.
        agent.term_overlap = False
        agent.term_overlap_again = False
        agent.term_index = 0
        agent.eta = 1.0
        agent.term_last_pos = agent.p.copy()
