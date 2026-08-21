"""Shared per-step context object passed to every policy component.

Keeping inputs in one struct lets each plugin (selector, yielder, lifecycle,
negotiator) read what it needs without coupling to the orchestrator's
internals or to a deep inheritance chain.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional

import numpy as np

PAD_CENTER = np.array([0.0, 0.0])


@dataclass
class Context:
    """Per-step inputs visible to policy components."""

    agent_list: List[Any]
    num_moving_drones: int
    target: Optional[List[Any]] = None
    target_reached: Optional[List[bool]] = None
    step: int = 0
    verbose: bool = False
    K: int = 0
    active_idx: Optional[int] = None  # index of the currently allowed drone
