"""Tests for openarm_fk.py — FK library for openarm_tesollo_sensor."""

import numpy as np
import pytest

from openarm_fk import (
    apply_action_delta,
    fk_left_tcp,
    fk_right_palm,
    fk_trajectory,
    pose_to_action_delta,
)


# ---- helpers ----

def _assert_valid_se3(T: np.ndarray) -> None:
    R = T[:3, :3]
    assert abs(np.linalg.det(R) - 1.0) < 1e-10, f"det(R)={np.linalg.det(R)}"
    assert np.max(np.abs(R @ R.T - np.eye(3))) < 1e-10, "R not orthogonal"
    assert np.allclose(T[3], [0, 0, 0, 1]), f"bottom row={T[3]}"


# ---- fixtures ----

@pytest.fixture
def q_zero():
    return np.zeros(7)


@pytest.fixture
def q_nonzero():
    return np.array([0.1, 0.3, 0.2, 0.5, 0.1, 0.3, 0.2])


# ---- zero-pose geometry ----

def test_zero_pose_right_left_symmetry(q_zero):
    T_r = fk_right_palm(q_zero)
    T_l = fk_left_tcp(q_zero)
    assert T_r[1, 3] < 0, "right palm should be at negative y"
    assert T_l[1, 3] > 0, "left tcp should be at positive y"


def test_zero_pose_valid_se3(q_zero):
    _assert_valid_se3(fk_right_palm(q_zero))
    _assert_valid_se3(fk_left_tcp(q_zero))


# ---- SO(3) validity at random configs ----

@pytest.mark.parametrize("seed", range(5))
def test_random_config_valid_se3(seed):
    rng = np.random.default_rng(seed)
    q = rng.uniform(-1.5, 1.5, 7)
    _assert_valid_se3(fk_right_palm(q))
    _assert_valid_se3(fk_left_tcp(q))


# ---- joint sensitivity ----

def test_all_joints_affect_eef_at_nonzero_config(q_nonzero):
    T_base = fk_right_palm(q_nonzero)
    for j in range(7):
        q = q_nonzero.copy()
        q[j] += 0.1
        T = fk_right_palm(q)
        rot_diff = np.max(np.abs(T[:3, :3] - T_base[:3, :3]))
        assert rot_diff > 1e-4, f"joint {j+1} had no effect on EEF orientation"


# ---- pose_to_action_delta / apply_action_delta roundtrip ----

@pytest.mark.parametrize("seed", range(10))
def test_delta_roundtrip(seed):
    rng = np.random.default_rng(seed)
    q_a = rng.uniform(-1.0, 1.0, 7)
    q_b = q_a + rng.uniform(-0.1, 0.1, 7)
    T_a = fk_right_palm(q_a)
    T_b = fk_right_palm(q_b)

    delta = pose_to_action_delta(T_a, T_b)
    T_b_hat = apply_action_delta(T_a, delta)

    assert np.linalg.norm(T_b_hat[:3, 3] - T_b[:3, 3]) < 1e-12
    assert np.max(np.abs(T_b_hat[:3, :3] - T_b[:3, :3])) < 1e-12


def test_delta_zero_for_same_pose(q_nonzero):
    T = fk_right_palm(q_nonzero)
    delta = pose_to_action_delta(T, T)
    assert np.linalg.norm(delta) < 1e-12


# ---- fk_trajectory batch ----

def test_fk_trajectory_shape():
    rng = np.random.default_rng(0)
    q_traj = rng.uniform(-0.5, 0.5, (50, 7))
    assert fk_trajectory(q_traj, arm="right").shape == (50, 4, 4)
    assert fk_trajectory(q_traj, arm="left").shape == (50, 4, 4)


def test_fk_trajectory_matches_single(q_nonzero):
    T_single = fk_right_palm(q_nonzero)
    T_batch = fk_trajectory(q_nonzero[None], arm="right")[0]
    assert np.allclose(T_single, T_batch)


# ---- small motion scale sanity ----

def test_small_joint_step_small_taskspace_delta():
    q_a = np.array([0.0, 0.3, 0.0, 0.5, 0.0, 0.3, 0.0])
    q_b = q_a.copy()
    q_b[0] += 0.01
    delta = pose_to_action_delta(fk_right_palm(q_a), fk_right_palm(q_b))
    assert np.linalg.norm(delta[:3]) < 0.05


# ---- delta sign convention matches pour_mimic_math ----

def test_delta_translation_equals_position_difference():
    """action[0:3] == T_next.pos - T_curr.pos (world frame)."""
    q_a = np.array([0.1, 0.3, 0.2, 0.5, 0.1, 0.3, 0.2])
    q_b = q_a.copy()
    q_b[3] += 0.5
    T_a = fk_right_palm(q_a)
    T_b = fk_right_palm(q_b)
    delta = pose_to_action_delta(T_a, T_b)
    assert np.allclose(delta[:3], T_b[:3, 3] - T_a[:3, 3])


def test_delta_rotation_left_multiplication():
    """R_next == R_delta @ R_curr (left multiplication, world frame)."""
    q_a = np.array([0.1, 0.3, 0.2, 0.5, 0.1, 0.3, 0.2])
    q_b = q_a.copy()
    q_b[5] += 0.3
    T_a = fk_right_palm(q_a)
    T_b = fk_right_palm(q_b)
    delta = pose_to_action_delta(T_a, T_b)

    from scipy.spatial.transform import Rotation
    R_delta = Rotation.from_rotvec(delta[3:6]).as_matrix()
    R_reconstructed = R_delta @ T_a[:3, :3]
    assert np.allclose(R_reconstructed, T_b[:3, :3], atol=1e-12)
