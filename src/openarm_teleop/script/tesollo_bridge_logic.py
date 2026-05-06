"""Pure-logic helpers for the Tesollo bridge node.

Separated from ROS 2 so they can be unit-tested without a running middleware.
"""
from __future__ import annotations

import math
from typing import Sequence

JOINT_NAMES: list[str] = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

HAND_APPROACH_POSE: list[float] = [
    0.0, -1.57, -0.5, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
]

HAND_GRASP_POSE: list[float] = [
    +0.000, -1.570, +0.200, +1.500,  
    +0.000, +1.800, +1.200, +1.300,   
    +0.000, +1.750, +0.700, +1.300, 
    +0.000, +1.650, +0.700, +1.300,  
    +0.000, -0.000, +1.400, +1.300, 
]


def compute_alpha(
    leader_pos: float,
    open_pos: float,
    grasp_pos: float,
    invert: bool = False,
) -> float | None:
    """Normalise leader gripper position to alpha in [0.0, 1.0].

    Returns None when the input is invalid (NaN, inf) or the range is
    degenerate (open_pos == grasp_pos).
    """
    if math.isnan(leader_pos) or math.isinf(leader_pos):
        return None
    if abs(grasp_pos - open_pos) < 1e-9:
        return None
    raw = (leader_pos - open_pos) / (grasp_pos - open_pos)
    alpha = max(0.0, min(1.0, raw))
    if invert:
        alpha = 1.0 - alpha
    return alpha


def compute_target_positions(
    alpha: float,
    q_initial: Sequence[float],
    q_grasp: Sequence[float],
) -> list[float]:
    """Interpolate between initial and grasp joint positions.

    q = q_initial + alpha * (q_grasp - q_initial)
    """
    return [q0 + alpha * (q1 - q0) for q0, q1 in zip(q_initial, q_grasp)]
