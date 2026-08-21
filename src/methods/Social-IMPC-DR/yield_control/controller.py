"""Orchestrator that runs the four policy roles each step.

The controller does not implement *any* policy itself. It composes:

- one ``Lifecycle``  (one_way / round_trip)
- one ``Selector``   (closest_first / priority)
- one ``Yielder``    (freeze / orbit)
- zero or more ``Negotiator`` peers (expiry_guard, eta_switch, ...)
- an optional hysteresis flag (lock-in winner until they finish)

Each role can be swapped independently via the recipe in the scenario JSON.
The orchestrator preserves the public surface ``test.py`` already calls:
``cleanup_landed``, ``select_active_drone``, ``freeze_yielding``,
``get_released_drones``, ``reset_mpc``, ``update_idle_positions``,
``step_update``, ``all_finished``.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Set

import numpy as np

from .context import Context
from .lifecycles import Lifecycle
from .negotiators import Negotiator
from .selectors import Selector
from .yielders import Yielder


class PolicyYieldController:
    def __init__(
        self,
        selector: Selector,
        yielder: Yielder,
        lifecycle: Lifecycle,
        negotiators: Sequence[Negotiator] = (),
        use_hysteresis: bool = False,
    ) -> None:
        self.selector = selector
        self.yielder = yielder
        self.lifecycle = lifecycle
        self.negotiators: List[Negotiator] = list(negotiators)
        self.use_hysteresis = use_hysteresis

        self._active_idx: Optional[int] = None
        self._held_winner: Optional[int] = None
        self._held_method: Optional[str] = None

    # ------------------------------------------------------------------
    # Optional wiring (round-trip needs the mutable target list)
    # ------------------------------------------------------------------
    def bind(self, target_array) -> None:
        self.lifecycle.bind(target_array)

    # ------------------------------------------------------------------
    # Lifecycle hooks
    # ------------------------------------------------------------------
    def cleanup_landed(self, agent_list, target_reached, num_moving_drones, K):
        ctx = Context(
            agent_list=agent_list,
            num_moving_drones=num_moving_drones,
            target_reached=target_reached,
            K=K,
        )
        self.lifecycle.cleanup_landed(ctx)

    def step_update(self, agent_list, target_reached, num_moving_drones):
        # Decrement priority TTE only when the selector cares about it.
        if isinstance(self.selector, _TteAware) or self.selector.name == "priority":
            for j in range(num_moving_drones):
                a = agent_list[j]
                if not target_reached[j]:
                    if hasattr(a, "time_to_expiry") and a.time_to_expiry is not None:
                        a.time_to_expiry = max(0.0, a.time_to_expiry - 1.0)
                if hasattr(a, "tte_history"):
                    a.tte_history.append(a.time_to_expiry)

        ctx = Context(
            agent_list=agent_list,
            num_moving_drones=num_moving_drones,
            target_reached=target_reached,
        )
        self.lifecycle.step_update(ctx)

    def all_finished(self, target_reached, num_moving_drones):
        ctx = Context(
            agent_list=[],
            num_moving_drones=num_moving_drones,
            target_reached=target_reached,
        )
        result = self.lifecycle.all_finished(ctx)
        if result is None:
            return all(target_reached[:num_moving_drones])
        return bool(result)

    def print_negotiator_summaries(self) -> None:
        """Let optional negotiators print end-of-run diagnostics."""
        for negotiator in self.negotiators:
            summary = getattr(negotiator, "print_llm_summary", None)
            if callable(summary):
                summary()

    # ------------------------------------------------------------------
    # Per-step decision
    # ------------------------------------------------------------------
    def select_active_drone(self, agent_list, active_drones, step, verbose):
        ctx = Context(
            agent_list=agent_list,
            num_moving_drones=len(agent_list),
            step=step,
            verbose=verbose,
            active_idx=self._active_idx,
        )

        candidates = self.lifecycle.filter_inbound(list(active_drones), ctx)

        if not candidates:
            decision = {
                "allowed":  None,
                "yielding": set(),
                "method":   "no_candidates",
                "scores":   {},
            }
            self._active_idx = None
            return decision

        forced = self.lifecycle.force_yield(candidates, ctx)
        if forced is not None:
            self._active_idx = forced.get("allowed")
            return forced

        if len(candidates) == 1:
            idx = candidates[0]
            decision = {
                "allowed":  idx,
                "yielding": set(),
                "method":   self.selector.name,
                "scores":   {},
            }
            self._commit(decision)
            self._active_idx = idx
            return decision

        # Negotiators are peers; first non-None override wins.
        override = None
        for n in self.negotiators:
            d = n.override(candidates, ctx)
            if d is not None:
                override = d
                break

        decision = self._resolve(candidates, override, ctx)
        self._commit(decision)
        self._active_idx = decision.get("allowed")
        return decision

    def _resolve(
        self,
        candidates: List[int],
        override: Optional[Dict],
        ctx: Context,
    ) -> Dict:
        if not self.use_hysteresis:
            return override if override is not None else self.selector.select(candidates, ctx)

        # Expiry-guard always trumps the held winner.
        if override is not None and override.get("method") == "expiry_guard":
            return override

        if (
            self._held_winner is not None
            and self._held_winner in candidates
        ):
            return {
                "allowed":  self._held_winner,
                "yielding": {j for j in candidates if j != self._held_winner},
                "method":   self._held_method or self.selector.name,
                "scores":   self.selector.score(candidates, ctx),
            }

        if override is not None:
            return override
        return self.selector.select(candidates, ctx)

    def _commit(self, decision: Dict) -> None:
        self._held_winner = decision.get("allowed")
        self._held_method = decision.get("method")

    # ------------------------------------------------------------------
    # Yield application + MPC bookkeeping
    # ------------------------------------------------------------------
    def freeze_yielding(self, agent_list, yielding_drones):
        ctx = Context(
            agent_list=agent_list,
            num_moving_drones=len(agent_list),
            active_idx=self._active_idx,
        )
        self.yielder.apply(yielding_drones, ctx)

    @staticmethod
    def get_released_drones(process_indices, prev_yielding) -> List[int]:
        return [j for j in process_indices if j in prev_yielding]

    @staticmethod
    def reset_mpc(agent_list, released_indices, verbose) -> None:
        for j in released_indices:
            agent_list[j].pre_traj = np.tile(
                agent_list[j].p, (agent_list[j].K + 1, 1)
            )
            agent_list[j].v = np.zeros_like(agent_list[j].v)
            agent_list[j].state = np.append(agent_list[j].p, agent_list[j].v)
            agent_list[j].cost_index = agent_list[j].K
            if verbose:
                print(f"  Drone {j} released from holding — MPC reset")

    @staticmethod
    def update_idle_positions(
        agent_list, process_indices, target_reached, target, num_moving_drones
    ) -> None:
        for j in range(num_moving_drones):
            if j not in process_indices:
                if target_reached[j]:
                    agent_list[j].position = np.block(
                        [[agent_list[j].position], [target[j]]]
                    )
                else:
                    agent_list[j].position = np.block(
                        [[agent_list[j].position], [agent_list[j].p]]
                    )


class _TteAware:
    """Marker base; future TTE-aware selectors can subclass this so the
    controller knows to advance ``time_to_expiry`` each step without us
    hardcoding selector names. Currently unused but reserved for plugins."""

    pass
