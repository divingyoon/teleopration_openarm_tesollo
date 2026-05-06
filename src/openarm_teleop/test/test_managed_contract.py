"""Managed dataset contract checks for pour_v1_mimic."""

import numpy as np
import pytest

from db3_to_pour_mimic_hdf5 import validate_demo_payload
from pour_v1_mimic_contract import ACTION_DIM, ACTOR_OBSERVATION_DIM


def _make_demo(n_steps: int = 16) -> dict:
    obs = {
        "right_joint_pos": np.zeros((n_steps, 27), dtype=np.float32),
        "right_joint_vel": np.zeros((n_steps, 27), dtype=np.float32),
        "left_joint_pos": np.zeros((n_steps, 7), dtype=np.float32),
        "left_joint_vel": np.zeros((n_steps, 7), dtype=np.float32),
        "tip_force_norm": np.zeros((n_steps, 5), dtype=np.float32),
        "prev_actions": np.zeros((n_steps, ACTION_DIM), dtype=np.float32),
    }
    obs["actor_obs"] = np.zeros((n_steps, ACTOR_OBSERVATION_DIM), dtype=np.float32)
    return {
        "actions": np.zeros((n_steps, ACTION_DIM), dtype=np.float32),
        "obs": obs,
        "timestamps_ns": np.arange(n_steps, dtype=np.int64),
    }


def test_valid_demo_passes():
    validate_demo_payload(_make_demo())


def test_action_dim_mismatch_fails():
    demo = _make_demo()
    demo["actions"] = np.zeros((demo["actions"].shape[0], ACTION_DIM + 1), dtype=np.float32)
    with pytest.raises(ValueError):
        validate_demo_payload(demo)


def test_nan_fails_fast():
    demo = _make_demo()
    demo["obs"]["right_joint_pos"][0, 0] = np.nan
    with pytest.raises(ValueError):
        validate_demo_payload(demo)


def test_length_mismatch_fails():
    demo = _make_demo()
    demo["obs"]["tip_force_norm"] = np.zeros((demo["actions"].shape[0] - 1, 5), dtype=np.float32)
    with pytest.raises(ValueError):
        validate_demo_payload(demo)
