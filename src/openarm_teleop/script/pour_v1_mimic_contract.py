#!/usr/bin/env python3
"""Shared contracts for the pour_v1_mimic data and training pipeline."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable

import numpy as np

TASK_ID_EVAL = "Pour-Mimic-V1-v0"
TASK_ID_MIMIC = "Pour-Mimic-V1-Mimic-v0"
PRE_POUR_INTENT = "pre_pour_init"
PRE_POUR_TRUNCATE_SIGNAL = "align_done"

ACTION_DIM = 18
RIGHT_PALM_DELTA_SLICE = slice(0, 6)
RIGHT_HAND_CURL_SLICE = slice(6, 11)
LEFT_ARM_DELTA_SLICE = slice(11, 18)

OBSERVATION_DIM_BY_KEY = {
    "right_joint_pos": 27,  # right arm 7 + right hand 20
    "right_joint_vel": 27,
    "left_joint_pos": 7,
    "left_joint_vel": 7,
    "tip_force_norm": 5,
    "prev_actions": 18,
}
ACTOR_OBSERVATION_DIM = sum(OBSERVATION_DIM_BY_KEY.values())

SUBTASK_TERM_SIGNALS = ("grasp_done", "lift_done", "align_done", "pour_done")
SUBTASK_START_SIGNALS = ("grasp_start", "lift_start", "align_start", "pour_start")
PRE_POUR_REQUIRED_TERM_SIGNALS = ("grasp_done", "lift_done", "align_done", "pour_done")
PRE_POUR_REQUIRED_START_SIGNALS = ("pour_start",)

# Canonical DB3 topics for pour teleop collection.
REQUIRED_TOPICS = (
    "/openarm/left/joint_states",
    "/openarm/right/joint_states",
    "/dg5f_right/rj_dg_pospid/reference",
)
OPTIONAL_TOPICS = (
    "/dg5f_right/joint_states",
    "/tesollo/right/sensor",
    "/tf",
    "/tf_static",
)

# Nominal publish rates used by the quality gate to compute per-topic expected
# message counts. Topics publish at different natural frequencies, so comparing
# counts across topics (arm at ~1000 Hz vs hand reference at ~100 Hz) is
# meaningless. Each topic is evaluated against: expected = nominal_hz * duration_sec
TOPIC_NOMINAL_HZ: dict[str, float] = {
    "/openarm/left/joint_states": 1000.0,
    "/openarm/right/joint_states": 1000.0,
    "/dg5f_right/rj_dg_pospid/reference": 100.0,
}


def validate_action_tensor(actions: np.ndarray) -> None:
    if actions.ndim != 2:
        raise ValueError(f"actions must be rank-2, got shape={actions.shape}")
    if actions.shape[-1] != ACTION_DIM:
        raise ValueError(f"actions last dim must be {ACTION_DIM}, got {actions.shape[-1]}")
    if not np.isfinite(actions).all():
        raise ValueError("actions contains NaN or inf")


def extract_gripper_curl(actions: np.ndarray) -> np.ndarray:
    validate_action_tensor(actions)
    return actions[..., RIGHT_HAND_CURL_SLICE]


def validate_subtask_signals(signals: Iterable[str]) -> None:
    seen = set(signals)
    for required in SUBTASK_TERM_SIGNALS + SUBTASK_START_SIGNALS:
        if required not in seen:
            raise ValueError(f"missing subtask signal: {required}")


def resolve_phase2_ros2_bridge_scripts_path() -> str:
    """Resolve scripts path used by phase2 bridge contract tests.

    Priority:
    1) SIM2REAL_SCRIPTS_DIR env override
    2) /home/usr/rl_ws/sim2real/scripts
    3) /home/user/rl_ws/sim2real/scripts (legacy fallback)
    """
    override = os.environ.get("SIM2REAL_SCRIPTS_DIR")
    if override:
        return override

    preferred = Path("/home/usr/rl_ws/sim2real/scripts")
    if preferred.exists():
        return str(preferred)

    legacy = Path("/home/user/rl_ws/sim2real/scripts")
    return str(legacy)
