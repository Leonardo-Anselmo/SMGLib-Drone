"""Compatibility import for older scripts.

New final-submission code should import from ``trajectory_planner``.
"""

from trajectory_planner.llm_advisor import TrajectoryLLMAdvisor

__all__ = ["TrajectoryLLMAdvisor"]
