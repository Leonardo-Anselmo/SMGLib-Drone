"""Yielders apply the holding behaviour to non-winning drones each step.

Two peer implementations are provided:

- ``Freeze``  zero velocity in place (cheapest, used as baseline).
- ``Orbit``   circular holding pattern, with pre_traj prediction so the
              winner's MPC sees where holders will be.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np

from .context import Context


class Yielder:
    name: str = "yielder"

    def apply(self, yielding: Iterable[int], ctx: Context) -> None:
        raise NotImplementedError


class Freeze(Yielder):
    """Zero out velocity for every yielding drone."""

    name = "freeze"

    def apply(self, yielding: Iterable[int], ctx: Context) -> None:
        for j in yielding:
            ctx.agent_list[j].v = np.zeros(2)
            ctx.agent_list[j].state = np.append(
                ctx.agent_list[j].p, ctx.agent_list[j].v
            )


class Orbit(Yielder):
    """Each yielding drone flies a circular holding orbit.

    Per-drone state ``{center, angle}`` is initialised on first entry and
    re-initialised only if the drone has drifted well outside its stored
    ring (so brief yield flips don't snap the visual back to the start).

    If the orbit center would place the drone within
    ``orbit_radius + safe_distance`` of the active winner, the center is
    pushed outward so the entire ring stays clear.
    """

    name = "orbit"

    def __init__(
        self,
        orbit_radius: float = 0.7,
        orbit_speed: float = 0.15,
        safe_distance: float = 1.2,
    ) -> None:
        self.orbit_radius = orbit_radius
        self.orbit_speed = orbit_speed
        self.safe_distance = safe_distance
        self._state: dict = {}

    def apply(self, yielding: Iterable[int], ctx: Context) -> None:
        active_p = None
        if (
            ctx.active_idx is not None
            and ctx.active_idx < len(ctx.agent_list)
        ):
            active_p = ctx.agent_list[ctx.active_idx].p

        for j in yielding:
            needs_init = j not in self._state
            if not needs_init:
                drift = np.linalg.norm(
                    ctx.agent_list[j].p - self._state[j]["center"]
                )
                if drift > 1.5 * self.orbit_radius:
                    needs_init = True

            if needs_init:
                self._state[j] = {
                    "center": ctx.agent_list[j].p.copy(),
                    "angle":  0.0,
                }

            state = self._state[j]
            state["angle"] += self.orbit_speed

            if active_p is not None:
                vec = state["center"] - active_p
                dist_center = np.linalg.norm(vec)
                min_center_dist = self.orbit_radius + self.safe_distance
                if dist_center < min_center_dist:
                    push_dir = (
                        vec / dist_center
                        if dist_center > 1e-6 else np.array([1.0, 0.0])
                    )
                    state["center"] = active_p + push_dir * min_center_dist

            a = state["angle"]
            c = state["center"]
            r = self.orbit_radius

            new_p = c + r * np.array([np.cos(a), np.sin(a)])
            new_v = self.orbit_speed * r * np.array([-np.sin(a), np.cos(a)])

            ctx.agent_list[j].p     = new_p
            ctx.agent_list[j].v     = new_v
            ctx.agent_list[j].state = np.append(new_p, new_v)

            K = ctx.agent_list[j].K
            future_angles = a + self.orbit_speed * np.arange(1, K + 2)
            future_pos = c + r * np.column_stack(
                [np.cos(future_angles), np.sin(future_angles)]
            )
            ctx.agent_list[j].pre_traj = future_pos
