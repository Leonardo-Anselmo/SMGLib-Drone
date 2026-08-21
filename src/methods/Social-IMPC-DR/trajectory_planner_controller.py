"""Compatibility import for older scripts.

New final-submission code should import from ``trajectory_planner``.
"""

from trajectory_planner.baseline import TrajectoryPlannerController

__all__ = ["TrajectoryPlannerController"]
