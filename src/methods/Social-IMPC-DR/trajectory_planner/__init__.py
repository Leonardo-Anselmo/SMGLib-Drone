"""Trajectory planner track registry.

The final submission exposes several planner modes through the command
line. The baseline controller provides the default speed-scaled planner,
while the lookahead controller adds finite-horizon landing-order selection.
"""

from .baseline import TrajectoryPlannerController
from .lookahead import LookaheadTrajectoryPlannerController
from .registry import SUPPORTED_TRAJECTORY_MODES, build_trajectory_controller

__all__ = [
    "SUPPORTED_TRAJECTORY_MODES",
    "TrajectoryPlannerController",
    "LookaheadTrajectoryPlannerController",
    "build_trajectory_controller",
]
