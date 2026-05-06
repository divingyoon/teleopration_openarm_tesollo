"""Smoke tests for DB3->pour_v1_mimic HDF5 conversion primitives."""

from pathlib import Path

import h5py
import numpy as np
import pytest

from db3_to_pour_mimic_hdf5 import (
    _previous_actions,
    ObjectPoseConfig,
    TfEdge,
    TopicSample,
    build_demo,
    run_quality_gate,
    validate_demo_payload,
    write_hdf5,
)
from openarm_fk import apply_action_delta, fk_trajectory
from pour_v1_mimic_contract import ACTION_DIM, RIGHT_PALM_DELTA_SLICE


def _mk_samples(start_ns: int, step_ns: int, n: int, dim: int) -> list[TopicSample]:
    out = []
    for i in range(n):
        out.append(
            TopicSample(
                timestamp_ns=start_ns + i * step_ns,
                value=np.full((dim,), fill_value=float(i), dtype=np.float32),
            )
        )
    return out


def test_build_demo_produces_action_18d_and_valid_obs():
    samples = {
        "/openarm/left/joint_states": _mk_samples(0, 10_000_000, 20, 7),
        "/openarm/right/joint_states": _mk_samples(0, 10_000_000, 20, 7),
        "/dg5f_right/rj_dg_pospid/reference": _mk_samples(0, 10_000_000, 20, 20),
        "/dg5f_right/joint_states": _mk_samples(0, 10_000_000, 20, 20),
        "/tesollo/right/sensor": _mk_samples(0, 10_000_000, 20, 30),
    }
    demo = build_demo(samples, target_hz=100)
    validate_demo_payload(demo)
    assert demo["actions"].shape[1] == ACTION_DIM
    assert demo["obs"]["actor_obs"].shape[1] == 91
    assert demo["obs"]["right_joint_pos"].shape[1] == 27
    assert demo["obs"]["right_joint_vel"].shape[1] == 27
    assert demo["obs"]["left_joint_vel"].shape[1] == 7
    assert demo["obs"]["tip_force_norm"].shape[1] == 5
    assert demo["obs"]["prev_actions"].shape[1] == ACTION_DIM
    assert demo["obs"]["datagen_info"]["eef_pose"]["right"].shape[1:] == (4, 4)
    assert demo["obs"]["datagen_info"]["eef_pose"]["left"].shape[1:] == (4, 4)
    assert demo["obs"]["datagen_info"]["target_eef_pose"]["right"].shape[1:] == (4, 4)
    assert demo["obs"]["datagen_info"]["object_pose"]["source_cup"].shape[1:] == (4, 4)
    assert demo["obs"]["datagen_info"]["object_pose"]["target_cup"].shape[1:] == (4, 4)
    assert demo["obs"]["datagen_info"]["subtask_term_signals"]["align_done"].dtype == np.bool_
    assert demo["obs"]["datagen_info"]["subtask_start_signals"]["pour_start"].dtype == np.bool_


def _mixed_hz_samples(duration_sec: float = 12.0) -> dict:
    """Simulate realistic recording: arm at 1000 Hz, dg5f reference at 100 Hz."""
    arm_n = int(1000 * duration_sec)
    dg5f_n = int(100 * duration_sec)
    return {
        "/openarm/left/joint_states": _mk_samples(0, 1_000_000, arm_n, 7),
        "/openarm/right/joint_states": _mk_samples(0, 1_000_000, arm_n, 7),
        "/dg5f_right/rj_dg_pospid/reference": _mk_samples(0, 10_000_000, dg5f_n, 20),
        "/dg5f_right/joint_states": _mk_samples(0, 1_000_000, arm_n, 20),
        "/tesollo/right/sensor": _mk_samples(0, 1_000_000, arm_n, 30),
    }


def test_quality_gate_passes_with_mixed_hz_topics():
    """dg5f_reference at 100 Hz must not be penalized against arm topics at 1000 Hz."""
    samples = _mixed_hz_samples(duration_sec=12.0)
    result = run_quality_gate(samples, max_drop_rate=0.2, max_gap_sec=0.1)
    assert result["drop_by_topic"]["/dg5f_right/rj_dg_pospid/reference"] < 0.2


def test_quality_gate_fails_when_topic_genuinely_drops():
    """A topic with only 10% of its expected messages must still fail the gate."""
    samples = _mixed_hz_samples(duration_sec=12.0)
    # Simulate 90% message drop on left arm (1000 Hz → 100 messages)
    samples["/openarm/left/joint_states"] = samples["/openarm/left/joint_states"][:100]
    with pytest.raises(ValueError, match="quality gate failed"):
        run_quality_gate(samples, max_drop_rate=0.2, max_gap_sec=0.1)


def test_write_hdf5_roundtrip(tmp_path):
    samples = {
        "/openarm/left/joint_states": _mk_samples(0, 10_000_000, 20, 7),
        "/openarm/right/joint_states": _mk_samples(0, 10_000_000, 20, 7),
        "/dg5f_right/rj_dg_pospid/reference": _mk_samples(0, 10_000_000, 20, 20),
        "/dg5f_right/joint_states": _mk_samples(0, 10_000_000, 20, 20),
        "/tesollo/right/sensor": _mk_samples(0, 10_000_000, 20, 30),
    }
    demo = build_demo(samples, target_hz=100)
    output = Path(tmp_path) / "converted.hdf5"
    write_hdf5(output, [demo], [{"ok": True}], ["bag_a"])
    with h5py.File(output, "r") as h5:
        actions = h5["data/demo_0/actions"][:]
        assert actions.shape[1] == ACTION_DIM
        assert h5["data/demo_0/obs/actor_obs"][:].shape[1] == 91
        assert h5["data/demo_0/obs/right_joint_pos"][:].shape[1] == 27
        assert h5["data/demo_0/obs/right_joint_vel"][:].shape[1] == 27
        assert h5["data/demo_0/obs/tip_force_norm"][:].shape[1] == 5
        assert h5["data/demo_0/obs/right_hand_curl"][:].shape[1] == 5
        assert h5["data/demo_0/obs/datagen_info/eef_pose/right"][:].shape[1:] == (4, 4)
        assert h5["data/demo_0/obs/datagen_info/eef_pose/left"][:].shape[1:] == (4, 4)
        assert h5["data/demo_0/obs/datagen_info/target_eef_pose/right"][:].shape[1:] == (4, 4)
        assert h5["data/demo_0/obs/datagen_info/object_pose/source_cup"][:].shape[1:] == (4, 4)
        assert h5["data/demo_0/obs/datagen_info/object_pose/target_cup"][:].shape[1:] == (4, 4)
        assert h5["data/demo_0/obs/datagen_info/subtask_term_signals/align_done"][:].dtype == np.bool_
        assert h5["data/demo_0/obs/datagen_info/subtask_start_signals/pour_start"][:].dtype == np.bool_


def _nonzero_samples(n: int = 30) -> dict:
    """Samples with non-trivial (non-zero) joint angles for FK sensitivity tests."""
    rng = np.random.default_rng(0)

    def _mk_ramp(dim, n):
        base = rng.uniform(0.1, 0.5, dim)
        step = rng.uniform(0.001, 0.01, dim)
        return [
            TopicSample(
                timestamp_ns=i * 10_000_000,
                value=(base + step * i).astype(np.float32),
            )
            for i in range(n)
        ]

    return {
        "/openarm/left/joint_states": _mk_ramp(7, n),
        "/openarm/right/joint_states": _mk_ramp(7, n),
        "/dg5f_right/rj_dg_pospid/reference": _mk_ramp(20, n),
        "/dg5f_right/joint_states": _mk_ramp(20, n),
        "/tesollo/right/sensor": _mk_ramp(30, n),
    }


def test_action_palm_slice_is_taskspace_not_joint_delta():
    """actions[:,0:6] must be task-space deltas, not joint-space differences.

    Verify by checking that action[t] applied to eef_pose[t] recovers eef_pose[t+1].
    """
    samples = _nonzero_samples(n=30)
    demo = build_demo(samples, target_hz=100)

    actions = demo["actions"]                          # [T, 18]
    right_js = demo["obs"]["right_arm_joint_pos"]      # [T, 7]

    T_traj = fk_trajectory(right_js.astype(np.float64), arm="right")  # [T, 4, 4]

    palm_deltas = actions[:, RIGHT_PALM_DELTA_SLICE]   # [T, 6]

    # For every step t, applying action[t] to T[t] must recover T[t+1].
    max_pos_err = 0.0
    max_rot_err = 0.0
    for t in range(len(actions) - 1):
        T_pred = apply_action_delta(T_traj[t], palm_deltas[t].astype(np.float64))
        pos_err = np.linalg.norm(T_pred[:3, 3] - T_traj[t + 1][:3, 3])
        rot_err = np.max(np.abs(T_pred[:3, :3] - T_traj[t + 1][:3, :3]))
        max_pos_err = max(max_pos_err, pos_err)
        max_rot_err = max(max_rot_err, rot_err)

    assert max_pos_err < 1e-5, f"FK roundtrip position error too large: {max_pos_err:.2e}"
    assert max_rot_err < 1e-5, f"FK roundtrip rotation error too large: {max_rot_err:.2e}"


def test_action_palm_slice_differs_from_joint_delta():
    """actions[:,0:6] must NOT equal the old joint-space finite differences."""
    samples = _nonzero_samples(n=30)
    demo = build_demo(samples, target_hz=100)

    actions = demo["actions"]
    right_js = demo["obs"]["right_arm_joint_pos"]

    # Reconstruct what the old joint-space delta would have been.
    old_delta = np.zeros_like(right_js[:, :6])
    old_delta[1:] = right_js[1:, :6] - right_js[:-1, :6]

    palm_deltas = actions[:, RIGHT_PALM_DELTA_SLICE]

    # They must differ — confirms the new FK path is active.
    assert not np.allclose(palm_deltas, old_delta), (
        "actions[:,0:6] is still equal to joint-space delta — FK path not active"
    )


def test_datagen_target_eef_pose_is_next_step_shift():
    samples = _nonzero_samples(n=16)
    demo = build_demo(samples, target_hz=100)
    right = demo["obs"]["datagen_info"]["eef_pose"]["right"]
    right_target = demo["obs"]["datagen_info"]["target_eef_pose"]["right"]
    left = demo["obs"]["datagen_info"]["eef_pose"]["left"]
    left_target = demo["obs"]["datagen_info"]["target_eef_pose"]["left"]

    np.testing.assert_allclose(right_target[:-1], right[1:], rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(left_target[:-1], left[1:], rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(right_target[-1], right[-1], rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(left_target[-1], left[-1], rtol=1e-6, atol=1e-6)


def test_prev_actions_are_shifted_by_one():
    samples = _nonzero_samples(n=12)
    demo = build_demo(samples, target_hz=100)
    actions = demo["actions"]
    prev_actions = demo["obs"]["prev_actions"]
    expected = _previous_actions(actions)
    np.testing.assert_allclose(prev_actions, expected, rtol=1e-6, atol=1e-6)


def test_tip_force_norm_is_clamped_to_unit_interval():
    samples = _nonzero_samples(n=10)
    # Overwrite sensor to large force values -> norm should still clamp to 1.
    samples["/tesollo/right/sensor"] = _mk_samples(0, 10_000_000, 10, 30)
    demo = build_demo(samples, target_hz=100)
    tip = demo["obs"]["tip_force_norm"]
    assert tip.shape[1] == 5
    assert float(np.min(tip)) >= 0.0
    assert float(np.max(tip)) <= 1.0


def _T_xyz(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float32)
    T[:3, 3] = np.array([x, y, z], dtype=np.float32)
    return T


def test_object_pose_tf_mode_uses_tf_chain_and_fallback():
    samples = _nonzero_samples(n=8)
    samples["/tf_static"] = [
        TopicSample(
            timestamp_ns=0,
            value=[TfEdge(parent_frame="world", child_frame="camera_link", transform_pc=_T_xyz(1.0, 0.0, 0.0))],
        )
    ]
    samples["/tf"] = [
        TopicSample(
            timestamp_ns=0,
            value=[
                TfEdge(parent_frame="camera_link", child_frame="source_cup", transform_pc=_T_xyz(0.2, 0.0, 0.0)),
                # target_cup edge intentionally omitted -> static fallback expected
            ],
        )
    ]

    default_source = _T_xyz(9.0, 9.0, 9.0)
    default_target = _T_xyz(8.0, 8.0, 8.0)
    cfg = ObjectPoseConfig(
        mode="tf",
        source_cup_pose_w=default_source,
        target_cup_pose_w=default_target,
        tf_topic="/tf",
        source_cup_frame="source_cup",
        target_cup_frame="target_cup",
        tf_reference_frame="world",
    )
    demo = build_demo(samples, target_hz=100, object_pose_cfg=cfg)

    source_seq = demo["obs"]["datagen_info"]["object_pose"]["source_cup"]
    target_seq = demo["obs"]["datagen_info"]["object_pose"]["target_cup"]

    # world->source = world->camera @ camera->source
    expected_source = _T_xyz(1.2, 0.0, 0.0)
    np.testing.assert_allclose(source_seq[0], expected_source, rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(source_seq[-1], expected_source, rtol=1e-6, atol=1e-6)

    # target has no tf edge in sample -> static fallback
    np.testing.assert_allclose(target_seq[0], default_target, rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(target_seq[-1], default_target, rtol=1e-6, atol=1e-6)


def test_subtask_signals_are_cumulative_and_nonempty():
    samples = _nonzero_samples(n=20)
    demo = build_demo(samples, target_hz=100)
    num_steps = demo["actions"].shape[0]
    terms = demo["obs"]["datagen_info"]["subtask_term_signals"]
    starts = demo["obs"]["datagen_info"]["subtask_start_signals"]

    for key in ("grasp_done", "lift_done", "align_done", "pour_done"):
        sig = terms[key]
        assert sig.shape == (num_steps,)
        assert sig.dtype == np.bool_
        assert bool(np.any(sig))
        assert np.all(sig[1:] >= sig[:-1])

    for key in ("grasp_start", "lift_start", "align_start", "pour_start"):
        sig = starts[key]
        assert sig.shape == (num_steps,)
        assert sig.dtype == np.bool_
        assert bool(np.any(sig))
        assert np.all(sig[1:] >= sig[:-1])
