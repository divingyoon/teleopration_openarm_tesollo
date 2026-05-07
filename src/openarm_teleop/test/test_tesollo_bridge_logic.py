"""Unit tests for tesollo_bridge_logic.py

Tests run without ROS 2. conftest.py adds script/ to sys.path.
Run: cd src/openarm_teleop && python3 -m pytest test/ -v
"""
import pytest

from tesollo_bridge_logic import (
    HAND_APPROACH_POSE,
    HAND_GRASP_POSE,
    JOINT_NAMES,
    compute_alpha,
    compute_target_positions,
)

Q_INITIAL = HAND_APPROACH_POSE
Q_GRASP = HAND_GRASP_POSE


# ---------------------------------------------------------------------------
# compute_alpha
# ---------------------------------------------------------------------------

class TestComputeAlpha:
    def test_open_position_gives_zero(self):
        assert compute_alpha(0.0, 0.0, 1.0) == pytest.approx(0.0)

    def test_grasp_position_gives_one(self):
        assert compute_alpha(1.0, 0.0, 1.0) == pytest.approx(1.0)

    def test_mid_position_gives_half(self):
        assert compute_alpha(0.5, 0.0, 1.0) == pytest.approx(0.5)

    def test_clamp_below_zero(self):
        assert compute_alpha(-0.5, 0.0, 1.0) == pytest.approx(0.0)

    def test_clamp_above_one(self):
        assert compute_alpha(1.5, 0.0, 1.0) == pytest.approx(1.0)

    def test_invert_open_gives_one(self):
        assert compute_alpha(0.0, 0.0, 1.0, invert=True) == pytest.approx(1.0)

    def test_invert_grasp_gives_zero(self):
        assert compute_alpha(1.0, 0.0, 1.0, invert=True) == pytest.approx(0.0)

    def test_invert_mid_gives_half(self):
        assert compute_alpha(0.5, 0.0, 1.0, invert=True) == pytest.approx(0.5)

    def test_nan_input_returns_none(self):
        assert compute_alpha(float("nan"), 0.0, 1.0) is None

    def test_pos_inf_input_returns_none(self):
        assert compute_alpha(float("inf"), 0.0, 1.0) is None

    def test_neg_inf_input_returns_none(self):
        assert compute_alpha(float("-inf"), 0.0, 1.0) is None

    def test_degenerate_range_returns_none(self):
        assert compute_alpha(0.5, 1.0, 1.0) is None

    def test_non_zero_based_range(self):
        assert compute_alpha(0.5, 0.3, 0.7) == pytest.approx(0.5)

    def test_negative_range(self):
        assert compute_alpha(-0.5, -1.0, 0.0) == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# compute_target_positions
# ---------------------------------------------------------------------------

class TestComputeTargetPositions:
    def test_default_pose_lengths_match_joint_names(self):
        assert len(JOINT_NAMES) == 20
        assert len(HAND_APPROACH_POSE) == len(JOINT_NAMES)
        assert len(HAND_GRASP_POSE) == len(JOINT_NAMES)

    def test_alpha_zero_returns_initial(self):
        result = compute_target_positions(0.0, Q_INITIAL, Q_GRASP)
        for r, q0 in zip(result, Q_INITIAL):
            assert r == pytest.approx(q0)

    def test_alpha_one_returns_grasp(self):
        result = compute_target_positions(1.0, Q_INITIAL, Q_GRASP)
        for r, q1 in zip(result, Q_GRASP):
            assert r == pytest.approx(q1)

    def test_alpha_half_is_midpoint(self):
        result = compute_target_positions(0.5, Q_INITIAL, Q_GRASP)
        for r, q0, q1 in zip(result, Q_INITIAL, Q_GRASP):
            assert r == pytest.approx(q0 + 0.5 * (q1 - q0))

    def test_output_length_is_20(self):
        result = compute_target_positions(0.5, Q_INITIAL, Q_GRASP)
        assert len(result) == 20

    def test_rj_dg_1_2_grasp_is_negative(self):
        result = compute_target_positions(1.0, Q_INITIAL, Q_GRASP)
        idx = JOINT_NAMES.index("rj_dg_1_2")
        assert result[idx] < 0.0

    def test_alpha_zero_uses_requested_thumb_approach(self):
        result = compute_target_positions(0.0, Q_INITIAL, Q_GRASP)
        assert result[1] == pytest.approx(-1.57)
        assert result[2] == pytest.approx(-0.5)

    def test_alpha_one_uses_requested_index_grasp(self):
        result = compute_target_positions(1.0, Q_INITIAL, Q_GRASP)
        assert result[4:8] == pytest.approx([0.0, 1.8, 1.2, 1.3])

    def test_non_thumb_abduction_joints_stay_fixed_during_synergy(self):
        result = compute_target_positions(1.0, Q_INITIAL, Q_GRASP)
        for name in ("rj_dg_2_1", "rj_dg_3_1", "rj_dg_4_1", "rj_dg_5_1"):
            assert result[JOINT_NAMES.index(name)] == pytest.approx(0.0)
