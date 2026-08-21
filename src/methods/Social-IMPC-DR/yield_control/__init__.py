"""Yield-control track: flat plugin model.

The track 1 controller is now a single ``PolicyYieldController`` that
composes peer-level policy components instead of inheriting them.

Roles
-----
- ``Selector``     decides who proceeds to the pad
- ``Yielder``      decides how non-winners hold (freeze, orbit)
- ``Lifecycle``    governs landing/return cycles (one-way, round-trip)
- ``Negotiator``   may override the selector decision (expiry guard, ETA switch)

A scenario picks one of each role plus zero or more negotiators via the
``policy`` block in the JSON config; see ``configs/track_policy_*.json``.
"""

from .context import Context, PAD_CENTER
from .controller import PolicyYieldController
from .registry import build_policy_yield_controller, build_from_recipe

__all__ = [
    "Context",
    "PAD_CENTER",
    "PolicyYieldController",
    "build_policy_yield_controller",
    "build_from_recipe",
]
