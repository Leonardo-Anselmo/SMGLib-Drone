"""
Trajectory Planner Controller (Track 2's only controller).

Track 2's method: instead of choosing a single winner and forcing the
rest to yield (the Track 1 model), a planner pre-computes a per-drone
cruise speed (`Vmax`) so all drones fly simultaneously and arrive at the
central pad in priority order, separated by `unload_steps` and
`min_separation` so each drone finishes unloading before the next
arrives.

This file owns everything Track 2 needs:

- the round-trip lifecycle FSM (`INBOUND -> UNLOADING -> OUTBOUND -> DONE`),
- per-drone return points + home-pad rendering hint,
- target swapping helpers (`_park`, `_switch_target`, `bind`),
- TTE countdown for cargo-aware priority scoring,
- the speed-scaling planner (`_replan`) that pushes `Vmax` to each drone,
- a pad-busy safety net (freeze near-pad inbound drones while another
  drone is UNLOADING).

The yield/orbit/negotiation family (`PriorityManager`, `OrbitController`,
`NegotiationController`, plus their LLM extension) lived on the
historical Phase 1-4 chain and is represented by the final branch's
`yield_control/` module. The only base class kept here is
`LandingPadController`, which provides the MPC-side plumbing
(`cleanup_landed` defaults, `freeze_yielding`, `reset_mpc`,
`update_idle_positions`, `get_released_drones`, `step_update`).

Usage:
    controller = TrajectoryPlannerController(
        cargo_configs,
        return_points=[start positions, one per drone],
        n_trips=1,           # 1 = one-way; >=2 = shuttle
        unload_steps=5,
        max_speed=1.0,
        min_separation=1.0,
        safe_distance=1.2,
        nominal_speed=0.1,
    )
    controller.bind(target_array)
"""

import numpy as np

import SET
from landing_pad import LandingPadController, PAD_CENTER
from priority import priority_score


INBOUND   = 'INBOUND'
UNLOADING = 'UNLOADING'
OUTBOUND  = 'OUTBOUND'
DONE      = 'DONE'


class TrajectoryPlannerController(LandingPadController):
    """Speed-scaled simultaneous arrivals + round-trip lifecycle FSM."""

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def __init__(
        self,
        cargo_configs=None,
        return_points=None,
        n_trips=1,
        unload_steps=5,
        max_speed=1.0,
        min_separation=1.0,
        safe_distance=1.2,
        nominal_speed=0.1,
        llm_advisor=None,
    ):
        super().__init__()
        # FSM bookkeeping
        self.cargo_configs     = cargo_configs or []
        self._return_points    = [np.array(rp, dtype=float)
                                  for rp in (return_points or [])]
        self._n_trips          = max(1, int(n_trips))
        self._unload_steps     = max(0, int(unload_steps))
        self._state            = {}    # j -> state string
        self._unload_remaining = {}    # j -> int
        self._trips_done       = {}    # j -> int
        self._target_ref       = None  # bound from test.py before main loop
        self._initialized      = False

        # Planner knobs
        self._max_speed        = float(max_speed)
        self._min_separation   = float(min_separation)
        self._safe_distance    = float(safe_distance)
        self._nominal_speed    = float(nominal_speed)
        self._last_inbound     = frozenset()
        self._last_scores      = {}
        self._last_schedule    = {}    # j -> {"T_arrive", "Vmax", "rank"}
        self._planner_dirty    = True
        self._llm_advisor      = llm_advisor

    # ------------------------------------------------------------------
    # External wiring
    # ------------------------------------------------------------------
    def bind(self, target_array):
        """Give the controller a handle to test.py's per-drone target list,
        so it can swap goals between PAD_CENTER and the return point."""
        self._target_ref = target_array

    def _ensure_init(self, num_moving_drones, agent_list=None):
        if self._initialized:
            return
        for j in range(num_moving_drones):
            self._state[j]            = INBOUND
            self._unload_remaining[j] = 0
            self._trips_done[j]       = 0
        # Stamp each drone's home pad onto its uav for visualisation only
        # when this is a true shuttle scenario (n_trips >= 2). One-way
        # scenarios (n_trips == 1) functionally still use self._return_points
        # for the OUTBOUND leg, but we leave agent.home_pad as None so the
        # renderer doesn't draw a stray home-pad circle at the start
        # position (which would just sit on top of the start-square marker).
        if agent_list is not None and self._n_trips >= 2:
            for j in range(min(num_moving_drones, len(agent_list))):
                if j < len(self._return_points):
                    agent_list[j].home_pad = self._return_points[j].copy()
        self._initialized = True

    # ------------------------------------------------------------------
    # Planner: rank inbound drones, assign T_arrive + Vmax
    # ------------------------------------------------------------------
    def _replan(self, agent_list, inbound, step):
        """Compute per-drone arrival slots and speed caps."""
        if not inbound:
            self._last_schedule = {}
            return

        # 1) Score inbound drones with the same priority function as Track 1.
        info = []
        for j in inbound:
            a = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            score = priority_score(
                cargo_type=getattr(a, "cargo_type", "equipment"),
                time_to_expiry=float(getattr(a, "time_to_expiry", 300.0)),
                distance_to_pad=dist,
                patient_acuity=getattr(a, "patient_acuity", "routine"),
            )
            info.append({"idx": j, "dist": dist, "score": score})

        if self._llm_advisor is not None:
            adjusted_scores = self._llm_advisor.maybe_adjust_scores(
                agent_list, info, step
            )
            if adjusted_scores is not None:
                for d in info:
                    d["score"] = adjusted_scores[d["idx"]]

        self._last_scores = {d["idx"]: d["score"] for d in info}

        # 2) Rank by score descending (higher priority first).
        info.sort(key=lambda d: d["score"], reverse=True)

        # 3) Assign slot times and speed caps with three floors.
        #    T >= d/max_speed
        #    T >= T_prev + unload_steps
        #    T >= T_prev + min_separation/v_prev
        schedule = {}
        T_prev = 0.0
        v_prev = self._max_speed
        for rank, d in enumerate(info):
            j = d["idx"]
            dist = d["dist"]
            T_phys = dist / max(self._max_speed, 1e-6)
            T_pad  = T_prev + self._unload_steps
            T_sep  = T_prev + self._min_separation / max(v_prev, 1e-6)
            T_arrive = max(T_phys,
                           T_pad if rank > 0 else 0.0,
                           T_sep if rank > 0 else 0.0)
            v_assign = min(dist / max(T_arrive, 1e-6), self._max_speed)
            schedule[j] = {"T_arrive": T_arrive, "Vmax": v_assign,
                           "rank": rank, "dist": dist}
            T_prev = T_arrive
            v_prev = v_assign

        # 4) Push speed caps to MPC.
        for j, sch in schedule.items():
            agent_list[j].Vmax = sch["Vmax"]

        # 5) OUTBOUND drones are unconstrained by pad contention.
        for j in range(len(agent_list)):
            if self._state.get(j) == OUTBOUND:
                agent_list[j].Vmax = self._max_speed

        self._last_schedule = schedule
        self._last_inbound  = frozenset(inbound)
        self._planner_dirty = False

        if step % 25 == 1:
            ranked = sorted(schedule.items(), key=lambda kv: kv[1]["rank"])
            summary = ", ".join(
                f"D{j}: v={s['Vmax']:.3f} T={s['T_arrive']:.1f}"
                for j, s in ranked
            )
            print(f"  [Planner] step {step} schedule -> {summary}")

        if self._llm_advisor is not None:
            self._llm_advisor.maybe_explain_schedule(
                agent_list, info, schedule, step
            )

    # ------------------------------------------------------------------
    # Override: planner mode -- everyone flies; safety-net freezes near-pad
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        self._ensure_init(len(agent_list), agent_list)
        inbound = [j for j in active_drones
                   if self._state.get(j, INBOUND) == INBOUND]

        if frozenset(inbound) != self._last_inbound or self._planner_dirty:
            self._replan(agent_list, inbound, step)

        # Safety net only while the pad is occupied by an unloading drone.
        pad_busy = any(s == UNLOADING for s in self._state.values())
        yielding = set()
        if pad_busy:
            for j in inbound:
                d_pad = float(np.linalg.norm(agent_list[j].p - PAD_CENTER))
                if d_pad < self._safe_distance:
                    yielding.add(j)

        return {
            "allowed":  None,  # planner does not use single-winner routing
            "yielding": yielding,
            "method":   "planner",
            "scores":   dict(self._last_scores),
        }

    # ------------------------------------------------------------------
    # Override: do not teleport on landing -- manage round-trip FSM
    # ------------------------------------------------------------------
    def cleanup_landed(self, agent_list, target_reached, num_moving_drones, K):
        before = dict(self._state)
        self._ensure_init(num_moving_drones, agent_list)

        for j in range(num_moving_drones):
            if not target_reached[j]:
                continue
            state = self._state[j]

            if state == INBOUND:
                # Just touched down at the pad - start the unload timer.
                self._state[j]            = UNLOADING
                self._unload_remaining[j] = self._unload_steps
                self._park(agent_list[j], agent_list[j].p, K)
                print(f"  [Lifecycle] Drone {j}: INBOUND -> UNLOADING "
                      f"(trip {self._trips_done[j] + 1}/{self._n_trips})")

            elif state == OUTBOUND:
                # Just reached the return point - count the trip.
                self._trips_done[j] += 1
                home = (self._return_points[j]
                        if j < len(self._return_points) else PAD_CENTER)
                if self._trips_done[j] >= self._n_trips:
                    self._state[j] = DONE
                    # Park at home pad (NOT teleport off-screen) so the
                    # drone is rendered resting where it belongs and acts
                    # as a static obstacle for any drones still flying.
                    self._park(agent_list[j], home, K)
                    print(f"  [Lifecycle] Drone {j}: OUTBOUND -> DONE "
                          f"(parked at home {home.tolist()}, "
                          f"completed {self._trips_done[j]} trips)")
                else:
                    # Begin the next inbound leg.
                    self._state[j] = INBOUND
                    self._switch_target(j, agent_list[j], target_reached,
                                        PAD_CENTER)
                    print(f"  [Lifecycle] Drone {j}: OUTBOUND -> INBOUND "
                          f"(starting trip {self._trips_done[j] + 1})")

            elif state == DONE:
                # Keep parked at home pad each step.
                home = (self._return_points[j]
                        if j < len(self._return_points) else PAD_CENTER)
                self._park(agent_list[j], home, K)

            # UNLOADING: handled by step_update countdown.

        # Mark the planner dirty if any FSM transitions fired this step,
        # and refresh OUTBOUND speed caps.
        if any(self._state.get(j) != before.get(j) for j in range(num_moving_drones)):
            self._planner_dirty = True
            for j in range(num_moving_drones):
                if self._state.get(j) == OUTBOUND:
                    agent_list[j].Vmax = self._max_speed

    # ------------------------------------------------------------------
    # Override: TTE countdown + unload countdown + outbound dispatch
    # ------------------------------------------------------------------
    def step_update(self, agent_list, target_reached, num_moving_drones):
        # TTE countdown for cargo-aware priority scoring (the planner reads
        # `time_to_expiry` inside `_replan` via `priority_score`). Was
        # previously inherited from `PriorityManager.step_update` on the
        # Phase 1-6 chain; inlined here so this controller stays
        # self-contained.
        for j in range(num_moving_drones):
            if not target_reached[j]:
                agent_list[j].time_to_expiry = max(
                    0.0, agent_list[j].time_to_expiry - 1.0
                )
            agent_list[j].tte_history.append(agent_list[j].time_to_expiry)

        before = dict(self._state)
        self._ensure_init(num_moving_drones, agent_list)

        for j in range(num_moving_drones):
            if self._state[j] != UNLOADING:
                continue
            self._unload_remaining[j] -= 1
            if self._unload_remaining[j] <= 0:
                self._state[j] = OUTBOUND
                rp = (self._return_points[j]
                      if j < len(self._return_points) else PAD_CENTER)
                self._switch_target(j, agent_list[j], target_reached, rp)
                print(f"  [Lifecycle] Drone {j}: UNLOADING -> OUTBOUND "
                      f"(heading to {rp.tolist()})")

        if any(self._state.get(j) != before.get(j) for j in range(num_moving_drones)):
            self._planner_dirty = True
            for j in range(num_moving_drones):
                if self._state.get(j) == OUTBOUND:
                    agent_list[j].Vmax = self._max_speed

    # ------------------------------------------------------------------
    # Override: only end the simulation when EVERY drone is DONE
    # ------------------------------------------------------------------
    def all_finished(self, target_reached, num_moving_drones):
        if not self._initialized:
            return False
        return all(self._state.get(j) == DONE
                   for j in range(num_moving_drones))

    # ------------------------------------------------------------------
    # FSM helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _park(agent, position, K):
        """Freeze the agent in place at `position` (used during UNLOADING / DONE)."""
        agent.p     = np.array(position, dtype=float)
        agent.v     = np.zeros(2)
        agent.state = np.append(agent.p, agent.v)
        agent.pre_traj = np.tile(agent.p, (K + 1, 1))

    def _switch_target(self, j, agent, target_reached, new_target):
        """Switch this drone's goal to `new_target` and re-arm MPC."""
        new_target = np.array(new_target, dtype=float)
        agent.change_target(new_target.copy())
        if self._target_ref is not None:
            self._target_ref[j] = new_target.copy()
        # IMPORTANT: run.run_one_agent re-asserts SET.target[j] every MPC
        # step, so we must update the SET-level target too -- otherwise the
        # change_target() above is overwritten on the next step and the
        # drone keeps trying to reach the original goal.
        if j < len(SET.target):
            SET.target[j] = new_target.copy()
        target_reached[j] = False
        # Warm-start the MPC trajectory from the current position so the
        # first solve doesn't dart backwards toward the old goal.
        agent.pre_traj  = np.tile(agent.p, (agent.K + 1, 1))
        agent.cost_index = agent.K
        # Clear the MPC's "I've converged on the old target" termination
        # flags. Without this reset the optimiser thinks it is already at
        # goal and refuses to plan a fresh trajectory toward `new_target`.
        agent.term_overlap       = False
        agent.term_overlap_again = False
        agent.term_index         = 0
        agent.eta                = 1.0
        agent.term_last_pos      = agent.p.copy()
