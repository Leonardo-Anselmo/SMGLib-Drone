"""Factories for trajectory-planner modes.

Mode names are part of the final submission interface:

- ``baseline``: current speed-scaled planner.
- ``llm``: reserved for Shariq's trajectory-planner method. For now this
  runs the baseline planner with the existing explanation advisor hook.
- ``lookahead``: reserved for Leonardo's upgraded look-ahead planner.
- ``compare_all``: reserved for evaluation runs across all planner modes.
  Until the missing modes land, it runs the baseline controller.
"""

from .baseline import TrajectoryPlannerController
from .lookahead import LookaheadTrajectoryPlannerController
from .llm_advisor import TrajectoryLLMAdvisor


SUPPORTED_TRAJECTORY_MODES = ("baseline", "llm", "lookahead", "compare_all")


def build_trajectory_controller(mode, cargo_configs, planner_params, target, ini_x, num_moving_drones):
    """Build the controller selected by the trajectory-planner command mode."""
    mode = mode or "baseline"
    if mode not in SUPPORTED_TRAJECTORY_MODES:
        supported = ", ".join(SUPPORTED_TRAJECTORY_MODES)
        raise ValueError(f"Unknown trajectory planner mode `{mode}`. Supported modes: {supported}")

    controller_cls = TrajectoryPlannerController
    if mode == "lookahead":
        controller_cls = LookaheadTrajectoryPlannerController

    if mode == "compare_all":
        print(
            "[TrajectoryPlanner] compare_all currently runs the baseline only; "
            "llm and lookahead modes will be included when those planners are implemented."
        )

    llm_advisor = None
    llm_params = planner_params.get("llm_advisor")
    if mode == "llm" and llm_params is None:
        llm_params = {"enabled": True, "mode": "explain"}
        print(
            "[TrajectoryPlanner] llm mode currently uses the baseline planner "
            "with the explanation advisor hook."
        )
    if llm_params and llm_params.get("enabled", False):
        llm_advisor = TrajectoryLLMAdvisor(
            mode=llm_params.get("mode", "explain"),
            model=llm_params.get("model"),
            cache_steps=llm_params.get("cache_steps", 25),
        )

    controller = controller_cls(
        cargo_configs,
        return_points=planner_params.get(
            "return_points",
            [ini_x[i] for i in range(num_moving_drones)],
        ),
        n_trips=planner_params.get("n_trips", 1),
        unload_steps=planner_params.get("unload_steps", 5),
        max_speed=planner_params.get("max_speed", 1.0),
        min_separation=planner_params.get("min_separation", 1.0),
        safe_distance=planner_params.get("safe_distance", 1.2),
        nominal_speed=planner_params.get("nominal_speed", 0.1),
        llm_advisor=llm_advisor,
    )
    controller.bind(target)
    return controller
