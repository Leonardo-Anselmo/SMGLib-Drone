"""Recipe -> ``PolicyYieldController`` builder.

Scenarios pick policy components by name in the JSON ``policy`` block::

    "policy": {
        "selector":       "priority",      // or "closest_first"
        "yielder":        "orbit",         // or "freeze"
        "lifecycle":      "round_trip",    // or "one_way"
        "negotiators":    ["expiry_guard", "eta_switch"],
        "use_hysteresis": true,
        "orbit_radius":   0.7,
        "orbit_speed":    0.15,
        "safe_distance":  1.2,
        "nominal_speed":  0.1,
        "eta_threshold":  0.15,
        "n_trips":        2,
        "unload_steps":   5
    }

The factory keeps a flat list of registered names so adding a new policy
is a one-liner change here plus the implementation file. There is no
inheritance chain.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

from .controller import PolicyYieldController
from .lifecycles import Lifecycle, OneWay, RoundTrip
from .negotiators import EtaSwitch, ExpiryGuard, LLMNegotiator, Negotiator
from .selectors import ClosestFirst, Priority, Selector
from .yielders import Freeze, Orbit, Yielder

SELECTORS: Dict[str, type] = {
    "closest_first": ClosestFirst,
    "priority":      Priority,
}

YIELDERS: Dict[str, type] = {
    "freeze": Freeze,
    "orbit":  Orbit,
}

LIFECYCLES: Dict[str, type] = {
    "one_way":    OneWay,
    "round_trip": RoundTrip,
}

NEGOTIATORS: Dict[str, type] = {
    "expiry_guard": ExpiryGuard,
    "eta_switch":   EtaSwitch,
    "llm_negotiator": LLMNegotiator,
}


def _make_selector(name: str) -> Selector:
    if name not in SELECTORS:
        raise ValueError(
            f"Unknown selector '{name}'. Registered: {sorted(SELECTORS)}"
        )
    return SELECTORS[name]()


def _make_yielder(name: str, params: Dict[str, Any]) -> Yielder:
    if name not in YIELDERS:
        raise ValueError(
            f"Unknown yielder '{name}'. Registered: {sorted(YIELDERS)}"
        )
    if name == "orbit":
        return Orbit(
            orbit_radius=params.get("orbit_radius",  0.7),
            orbit_speed=params.get("orbit_speed",    0.15),
            safe_distance=params.get("safe_distance", 1.2),
        )
    return YIELDERS[name]()


def _make_lifecycle(name: str, params: Dict[str, Any]) -> Lifecycle:
    if name not in LIFECYCLES:
        raise ValueError(
            f"Unknown lifecycle '{name}'. Registered: {sorted(LIFECYCLES)}"
        )
    if name == "round_trip":
        return RoundTrip(
            return_points=params.get("return_points"),
            n_trips=params.get("n_trips", 2),
            unload_steps=params.get("unload_steps", 5),
        )
    return LIFECYCLES[name]()


def _make_negotiator(name: str, params: Dict[str, Any]) -> Negotiator:
    if name not in NEGOTIATORS:
        raise ValueError(
            f"Unknown negotiator '{name}'. Registered: {sorted(NEGOTIATORS)}"
        )
    if name == "expiry_guard":
        return ExpiryGuard(nominal_speed=params.get("nominal_speed", 0.1))
    if name == "eta_switch":
        return EtaSwitch(
            nominal_speed=params.get("nominal_speed", 0.1),
            eta_threshold=params.get("eta_threshold", 0.15),
        )
    if name == "llm_negotiator":
        return LLMNegotiator(
            nominal_speed=params.get("nominal_speed", 0.1),
            llm_model=params.get("llm_model", "claude-haiku-4-5-20251001"),
            llm_cache_steps=params.get("llm_cache_steps", 10),
            use_llm_hook=params.get("use_llm_hook", True),
            use_llm_score=params.get("use_llm_score", False),
        )
    return NEGOTIATORS[name]()


def build_from_recipe(
    recipe: Dict[str, Any],
    target=None,
) -> PolicyYieldController:
    """Build a ``PolicyYieldController`` from a parsed JSON ``policy`` block."""

    selector_name = recipe.get("selector", "closest_first")
    yielder_name = recipe.get("yielder", "freeze")
    lifecycle_name = recipe.get("lifecycle", "one_way")
    negotiator_names: Sequence[str] = recipe.get("negotiators", []) or []
    use_hysteresis = bool(recipe.get("use_hysteresis", False))

    selector = _make_selector(selector_name)
    yielder = _make_yielder(yielder_name, recipe)
    lifecycle = _make_lifecycle(lifecycle_name, recipe)
    negotiators = [_make_negotiator(n, recipe) for n in negotiator_names]

    controller = PolicyYieldController(
        selector=selector,
        yielder=yielder,
        lifecycle=lifecycle,
        negotiators=negotiators,
        use_hysteresis=use_hysteresis,
    )
    if target is not None:
        controller.bind(target)
    return controller


def build_policy_yield_controller(
    target=None,
    policy_recipe: Optional[Dict[str, Any]] = None,
) -> PolicyYieldController:
    """Single integration point used by ``test.py``.

    A scenario without a ``policy`` block falls back to the closest-first
    baseline (no priority, no orbit, no negotiation, no round-trip),
    which matches the legacy ``LandingPadController`` default.
    """
    recipe = policy_recipe or {
        "selector":  "closest_first",
        "yielder":   "freeze",
        "lifecycle": "one_way",
    }
    return build_from_recipe(recipe, target=target)


def list_registered() -> Dict[str, List[str]]:
    """Diagnostics helper: return the names every role currently exposes."""
    return {
        "selectors":   sorted(SELECTORS),
        "yielders":    sorted(YIELDERS),
        "lifecycles":  sorted(LIFECYCLES),
        "negotiators": sorted(NEGOTIATORS),
    }
