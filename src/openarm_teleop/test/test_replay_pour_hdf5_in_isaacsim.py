from pathlib import Path

import h5py
import numpy as np
import pytest

from replay_pour_hdf5_in_isaacsim import (
    EXPECTED_JOINT_NAMES,
    LEFT_GRIPPER_JOINT_NAMES,
    ReplayDatasetError,
    append_optional_left_gripper,
    build_dataset_joint_positions,
    find_missing_joints,
    load_replay_trajectory,
)


def _write_minimal_demo(path: Path, *, right_hand_shape: tuple[int, int] = (3, 20)) -> None:
    with h5py.File(path, "w") as h5:
        demo = h5.create_group("data/demo_0")
        obs = demo.create_group("obs")
        obs.create_dataset(
            "right_arm_joint_pos",
            data=np.arange(3 * 7, dtype=np.float32).reshape(3, 7),
        )
        obs.create_dataset(
            "right_hand_joint_pos",
            data=np.arange(np.prod(right_hand_shape), dtype=np.float32).reshape(right_hand_shape),
        )
        obs.create_dataset(
            "left_arm_joint_pos",
            data=np.arange(100, 100 + 3 * 7, dtype=np.float32).reshape(3, 7),
        )
        demo.create_dataset("timestamps_ns", data=np.array([10, 20, 30], dtype=np.int64))


def test_build_dataset_joint_positions_uses_expected_order() -> None:
    right_arm = np.full((2, 7), 1.0, dtype=np.float32)
    right_hand = np.full((2, 20), 2.0, dtype=np.float32)
    left_arm = np.full((2, 7), 3.0, dtype=np.float32)

    positions = build_dataset_joint_positions(right_arm, right_hand, left_arm)

    assert positions.shape == (2, len(EXPECTED_JOINT_NAMES))
    np.testing.assert_array_equal(positions[:, :7], right_arm)
    np.testing.assert_array_equal(positions[:, 7:27], right_hand)
    np.testing.assert_array_equal(positions[:, 27:34], left_arm)


def test_load_replay_trajectory_reads_demo_and_timestamps(tmp_path: Path) -> None:
    dataset = tmp_path / "demo.hdf5"
    _write_minimal_demo(dataset)

    trajectory = load_replay_trajectory(dataset, "demo_0")

    assert trajectory.positions.shape == (3, 34)
    assert trajectory.timestamps_ns.tolist() == [10, 20, 30]
    assert trajectory.joint_names == EXPECTED_JOINT_NAMES


def test_load_replay_trajectory_prefers_robot_replay_joint_pos(tmp_path: Path) -> None:
    dataset = tmp_path / "robot_replay_demo.hdf5"
    with h5py.File(dataset, "w") as h5:
        demo = h5.create_group("data/demo_0")
        obs = demo.create_group("obs")
        obs.create_dataset("right_arm_joint_pos", data=np.zeros((2, 7), dtype=np.float32))
        obs.create_dataset("right_hand_joint_pos", data=np.zeros((2, 20), dtype=np.float32))
        obs.create_dataset("left_arm_joint_pos", data=np.zeros((2, 7), dtype=np.float32))
        replay_names = list(LEFT_GRIPPER_JOINT_NAMES) + EXPECTED_JOINT_NAMES
        replay = np.arange(2 * len(replay_names), dtype=np.float32).reshape(2, len(replay_names))
        ds = obs.create_dataset("robot_replay_joint_pos", data=replay)
        ds.attrs["joint_names"] = str(replay_names).replace("'", '"')

    trajectory = load_replay_trajectory(dataset, "demo_0")

    assert trajectory.joint_names == EXPECTED_JOINT_NAMES + list(LEFT_GRIPPER_JOINT_NAMES)
    expected_indices = [replay_names.index(name) for name in trajectory.joint_names]
    np.testing.assert_array_equal(trajectory.positions, replay[:, expected_indices])


def test_load_replay_trajectory_reorders_datasets_by_joint_name_attrs(tmp_path: Path) -> None:
    dataset = tmp_path / "shuffled_demo.hdf5"
    with h5py.File(dataset, "w") as h5:
        demo = h5.create_group("data/demo_0")
        obs = demo.create_group("obs")
        right_arm_names = ["openarm_right_joint2", "openarm_right_joint1"] + [
            f"openarm_right_joint{i}" for i in range(3, 8)
        ]
        right_arm = np.arange(7, dtype=np.float32)[None, [1, 0, 2, 3, 4, 5, 6]]
        ds = obs.create_dataset("right_arm_joint_pos", data=right_arm)
        ds.attrs["joint_names"] = str(right_arm_names).replace("'", '"')

        right_hand_names = ["rj_dg_1_2", "rj_dg_1_1"] + [
            f"rj_dg_{finger}_{joint}"
            for finger in range(1, 6)
            for joint in range(1, 5)
            if (finger, joint) not in {(1, 1), (1, 2)}
        ]
        right_hand = np.arange(20, dtype=np.float32)[None, [1, 0] + list(range(2, 20))]
        ds = obs.create_dataset("right_hand_joint_pos", data=right_hand)
        ds.attrs["joint_names"] = str(right_hand_names).replace("'", '"')

        left_arm = np.arange(100, 107, dtype=np.float32)[None, :]
        ds = obs.create_dataset("left_arm_joint_pos", data=left_arm)
        ds.attrs["joint_names"] = str([f"openarm_left_joint{i}" for i in range(1, 8)]).replace("'", '"')

    trajectory = load_replay_trajectory(dataset, "demo_0")

    np.testing.assert_array_equal(trajectory.positions[0, :7], np.arange(7, dtype=np.float32))
    np.testing.assert_array_equal(trajectory.positions[0, 7:27], np.arange(20, dtype=np.float32))


def test_append_optional_left_gripper_extends_replay_joint_order() -> None:
    positions = np.zeros((2, len(EXPECTED_JOINT_NAMES)), dtype=np.float32)
    left_gripper = np.array([[0.01, 0.01], [0.02, 0.02]], dtype=np.float32)

    replay_positions, joint_names = append_optional_left_gripper(
        positions,
        EXPECTED_JOINT_NAMES.copy(),
        left_gripper,
    )

    assert replay_positions.shape == (2, len(EXPECTED_JOINT_NAMES) + 2)
    assert joint_names[-2:] == list(LEFT_GRIPPER_JOINT_NAMES)
    np.testing.assert_array_equal(replay_positions[:, -2:], left_gripper)


def test_load_replay_trajectory_fails_fast_on_shape_mismatch(tmp_path: Path) -> None:
    dataset = tmp_path / "bad_demo.hdf5"
    _write_minimal_demo(dataset, right_hand_shape=(3, 19))

    with pytest.raises(ReplayDatasetError, match="right_hand_joint_pos"):
        load_replay_trajectory(dataset, "demo_0")


def test_find_missing_joints_reports_unavailable_usd_joints() -> None:
    available = EXPECTED_JOINT_NAMES[:-2] + ["unrelated_joint"]

    missing = find_missing_joints(EXPECTED_JOINT_NAMES, available)

    assert missing == EXPECTED_JOINT_NAMES[-2:]
