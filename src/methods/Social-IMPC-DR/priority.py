"""
Priority calculator for the Priority-Aware Multi-Drone Coordination system.

Computes a real-valued priority score per drone based on:
  - Cargo type (organ > blood_product > medication > equipment)
  - Time until expiration (lower remaining time = higher urgency)
  - Flight distance to landing pad (closer = slight advantage)
  - Patient acuity (critical > urgent > routine)

The score is dynamic: it is recomputed each simulation step as distance
and time-to-expiration change.

All tuneable values (factor weights, cargo weights, acuity scores,
normalisation caps) are loaded from ``configs/priority_config.json``.
"""

import json
import sys
from pathlib import Path

import numpy as np

# ── Load configuration from configs/priority_config.json ───────────────
_CONFIG_PATH = Path(__file__).resolve().parent / 'configs' / 'priority_config.json'

if not _CONFIG_PATH.exists():
    print(
        f"\n[ERROR] Required configuration file not found:\n"
        f"  {_CONFIG_PATH}\n\n"
        f"Please create 'configs/priority_config.json' with the priority\n"
        f"weights, cargo weights, acuity scores, and normalisation caps.\n"
        f"See README.md for the expected format.\n"
    )
    sys.exit(1)

with open(_CONFIG_PATH, 'r') as _f:
    _PRIORITY_CFG = json.load(_f)

# ── Cargo-type categorical weights ──────────────────────────────────────
CARGO_WEIGHTS = _PRIORITY_CFG['cargo_weights']

# ── Patient-acuity severity scores ──────────────────────────────────────
ACUITY_SCORES = _PRIORITY_CFG['acuity_scores']

# ── Default factor weights (sum to 1.0) ────────────────────────────────
DEFAULT_WEIGHTS = _PRIORITY_CFG['factor_weights']

# Maximum expected distance used to normalise the distance component.
_MAX_DISTANCE = _PRIORITY_CFG['max_distance']

# Maximum expected expiry time (seconds / steps) used for normalisation.
_MAX_EXPIRY = _PRIORITY_CFG['max_expiry']


def priority_score(cargo_type, time_to_expiry, distance_to_pad,
                   patient_acuity, weights=None):
    """Return a real-valued priority score in [0, 1].

    Higher score  →  higher priority  →  drone should proceed first.

    Parameters
    ----------
    cargo_type : str
        One of 'organ', 'blood_product', 'medication', 'equipment'.
    time_to_expiry : float
        Remaining time (steps or seconds) before the cargo expires.
        Lower values produce higher urgency.
    distance_to_pad : float
        Euclidean distance from the drone's current position to the pad.
    patient_acuity : str
        One of 'critical', 'urgent', 'routine'.
    weights : dict or None
        Optional override for factor weights (keys: cargo, expiry,
        distance, acuity).  Defaults to DEFAULT_WEIGHTS.

    Returns
    -------
    float
        Priority score in [0, 1].
    """
    w = weights if weights is not None else DEFAULT_WEIGHTS

    # 1. Cargo component ── direct lookup
    cargo_val = CARGO_WEIGHTS.get(cargo_type, 0.0)

    # 2. Expiry component ── inverse: less time left = higher score
    #    Clamp to [0, _MAX_EXPIRY] then invert.
    clamped_expiry = np.clip(time_to_expiry, 0.0, _MAX_EXPIRY)
    expiry_val = 1.0 - (clamped_expiry / _MAX_EXPIRY)

    # 3. Distance component ── inverse: closer = higher score
    clamped_dist = np.clip(distance_to_pad, 0.0, _MAX_DISTANCE)
    distance_val = 1.0 - (clamped_dist / _MAX_DISTANCE)

    # 4. Acuity component ── direct lookup
    acuity_val = ACUITY_SCORES.get(patient_acuity, 0.0)

    score = (w['cargo']    * cargo_val
           + w['expiry']   * expiry_val
           + w['distance'] * distance_val
           + w['acuity']   * acuity_val)

    return float(np.clip(score, 0.0, 1.0))


def rank_drones(drone_infos, weights=None):
    """Sort drones by priority score (descending).

    Parameters
    ----------
    drone_infos : list[dict]
        Each dict must contain keys:
            id, cargo_type, time_to_expiry, distance_to_pad, patient_acuity
    weights : dict or None
        Optional factor-weight override.

    Returns
    -------
    list[tuple[int, float]]
        List of (drone_id, score) sorted highest-first.
    """
    scored = []
    for d in drone_infos:
        s = priority_score(
            cargo_type=d['cargo_type'],
            time_to_expiry=d['time_to_expiry'],
            distance_to_pad=d['distance_to_pad'],
            patient_acuity=d['patient_acuity'],
            weights=weights,
        )
        scored.append((d['id'], s))
    scored.sort(key=lambda x: x[1], reverse=True)
    return scored
