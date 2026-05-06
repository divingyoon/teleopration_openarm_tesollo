"""Contracts for pour_v1_mimic task IDs, action layout and subtask signals."""

import numpy as np

from pour_v1_mimic_contract import (
    ACTION_DIM,
    ACTOR_OBSERVATION_DIM,
    OBSERVATION_DIM_BY_KEY,
    PRE_POUR_INTENT,
    PRE_POUR_REQUIRED_START_SIGNALS,
    PRE_POUR_REQUIRED_TERM_SIGNALS,
    PRE_POUR_TRUNCATE_SIGNAL,
    RIGHT_HAND_CURL_SLICE,
    SUBTASK_START_SIGNALS,
    SUBTASK_TERM_SIGNALS,
    TASK_ID_EVAL,
    TASK_ID_MIMIC,
    extract_gripper_curl,
    validate_action_tensor,
)


def test_task_ids_are_fixed():
    assert TASK_ID_EVAL == "Pour-Mimic-V1-v0"
    assert TASK_ID_MIMIC == "Pour-Mimic-V1-Mimic-v0"


def test_action_contract_dim_and_curl_slice():
    actions = np.zeros((8, ACTION_DIM), dtype=np.float32)
    validate_action_tensor(actions)
    curls = extract_gripper_curl(actions)
    assert curls.shape == (8, 5)
    assert RIGHT_HAND_CURL_SLICE.start == 6
    assert RIGHT_HAND_CURL_SLICE.stop == 11


def test_actor_observation_dim_matches_key_sum():
    assert ACTOR_OBSERVATION_DIM == sum(OBSERVATION_DIM_BY_KEY.values())
    assert ACTOR_OBSERVATION_DIM == 91


def test_subtask_signal_contract():
    assert SUBTASK_TERM_SIGNALS == ("grasp_done", "lift_done", "align_done", "pour_done")
    assert SUBTASK_START_SIGNALS == ("grasp_start", "lift_start", "align_start", "pour_start")


def test_pre_pour_constants():
    assert PRE_POUR_INTENT == "pre_pour_init"
    assert PRE_POUR_TRUNCATE_SIGNAL == "align_done"
    assert PRE_POUR_REQUIRED_TERM_SIGNALS == ("grasp_done", "lift_done", "align_done", "pour_done")
    assert PRE_POUR_REQUIRED_START_SIGNALS == ("pour_start",)
