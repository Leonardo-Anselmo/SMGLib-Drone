"""Selectors decide which active drone gets the pad each step.

Every selector returns a decision dict::

    {"allowed": int | None,
     "yielding": set[int],
     "method":  str,
     "scores":  dict[int, float]}

Selectors are peers; the controller chooses one per scenario and never
chains them.
"""

from __future__ import annotations

from typing import Dict, List, Set

import numpy as np

from priority import priority_score

from .context import Context, PAD_CENTER


class Selector:
    """Base class. Subclasses implement ``select`` and (optionally)
    ``score`` so other components (e.g. hysteresis) can reuse the
    per-drone scoring without re-running selection."""

    name: str = "selector"

    def select(self, candidates: List[int], ctx: Context) -> Dict:
        raise NotImplementedError

    def score(self, candidates: List[int], ctx: Context) -> Dict[int, float]:
        return {}


class ClosestFirst(Selector):
    """Pick the candidate closest to the pad. Distance is also the score."""

    name = "closest_first"

    def select(self, candidates: List[int], ctx: Context) -> Dict:
        scores = self.score(candidates, ctx)
        # Lower distance is better, so invert order when picking the winner.
        allowed_idx = min(scores, key=scores.get)

        if ctx.verbose and ctx.step % 20 == 0:
            ranked = sorted(scores.items(), key=lambda kv: kv[1])
            print(
                f"  Pad yielding: drone {allowed_idx} approaching "
                f"(dist={ranked[0][1]:.2f}), "
                f"others holding: {[d[0] for d in ranked[1:]]}"
            )

        yielding: Set[int] = {j for j in candidates if j != allowed_idx}
        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   self.name,
            "scores":   scores,
        }

    def score(self, candidates: List[int], ctx: Context) -> Dict[int, float]:
        return {
            j: float(np.linalg.norm(ctx.agent_list[j].p - PAD_CENTER))
            for j in candidates
        }


class Priority(Selector):
    """Pick the highest-priority candidate using ``priority_score``.

    Falls back to ``ClosestFirst`` when no candidate carries non-default
    cargo or acuity (otherwise priority is uninformative).
    """

    name = "priority"

    def __init__(self) -> None:
        self._fallback = ClosestFirst()

    def select(self, candidates: List[int], ctx: Context) -> Dict:
        has_priority = any(
            ctx.agent_list[j].cargo_type != "equipment"
            or ctx.agent_list[j].patient_acuity != "routine"
            for j in candidates
        )
        if not has_priority:
            return self._fallback.select(candidates, ctx)

        scores = self.score(candidates, ctx)
        allowed_idx = max(scores, key=scores.get)

        if ctx.verbose and ctx.step % 20 == 0:
            ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)
            print(
                f"  Priority yielding: drone {allowed_idx} proceeds "
                f"(score={ranked[0][1]:.3f}), "
                f"others: {[(i, f'{s:.3f}') for i, s in ranked[1:]]}"
            )

        yielding: Set[int] = {j for j in candidates if j != allowed_idx}
        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   self.name,
            "scores":   scores,
        }

    def score(self, candidates: List[int], ctx: Context) -> Dict[int, float]:
        scores: Dict[int, float] = {}
        for j in candidates:
            a = ctx.agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            scores[j] = priority_score(
                cargo_type=a.cargo_type,
                time_to_expiry=a.time_to_expiry,
                distance_to_pad=dist,
                patient_acuity=a.patient_acuity,
            )
        return scores
