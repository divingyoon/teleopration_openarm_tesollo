#!/usr/bin/env python3
"""Replay real pour HDF5 joint trajectories on OpenArm + Tesollo in IsaacSim."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import h5py
import numpy as np


RIGHT_ARM_JOINT_NAMES = tuple(f"openarm_right_joint{i}" for i in range(1, 8))
RIGHT_HAND_JOINT_NAMES = tuple(f"rj_dg_{finger}_{joint}" for finger in range(1, 6) for joint in range(1, 5))
LEFT_ARM_JOINT_NAMES = tuple(f"openarm_left_joint{i}" for i in range(1, 8))
LEFT_GRIPPER_JOINT_NAMES = ("openarm_left_finger_joint1", "openarm_left_finger_joint2")
EXPECTED_JOINT_NAMES = list(RIGHT_ARM_JOINT_NAMES + RIGHT_HAND_JOINT_NAMES + LEFT_ARM_JOINT_NAMES)
EXPECTED_ROBOT_REPLAY_JOINT_NAMES = EXPECTED_JOINT_NAMES + list(LEFT_GRIPPER_JOINT_NAMES)

DEFAULT_DATASET = Path("/home/user/rl_ws/teleopration_openarm_tesollo/datasets/pour_v1_test1.hdf5")
DEFAULT_USD = Path("/home/user/rl_ws/hdgp/assets/openarm_tesollo_sensor/openarm_tesollo_sensor.usd")


class ReplayDatasetError(ValueError):
    """Raised when an HDF5 demo cannot be replayed as a joint trajectory."""


@dataclass(frozen=True)
class ReplayTrajectory:
    """Joint replay data in Isaac articulation joint-name order."""

    positions: np.ndarray
    joint_names: list[str]
    timestamps_ns: np.ndarray | None

    @property
    def num_frames(self) -> int:
        return int(self.positions.shape[0])


def build_dataset_joint_positions(
    right_arm_joint_pos: np.ndarray,
    right_hand_joint_pos: np.ndarray,
    left_arm_joint_pos: np.ndarray,
) -> np.ndarray:
    """Concatenate the canonical HDF5 arrays into the 34-joint replay order."""

    arrays = (
        ("right_arm_joint_pos", right_arm_joint_pos, 7),
        ("right_hand_joint_pos", right_hand_joint_pos, 20),
        ("left_arm_joint_pos", left_arm_joint_pos, 7),
    )
    frame_count: int | None = None
    checked: list[np.ndarray] = []
    for name, values, expected_width in arrays:
        values = np.asarray(values, dtype=np.float32)
        if values.ndim != 2 or values.shape[1] != expected_width:
            raise ReplayDatasetError(
                f"{name} must have shape (N,{expected_width}); got {tuple(values.shape)}"
            )
        if frame_count is None:
            frame_count = int(values.shape[0])
        elif int(values.shape[0]) != frame_count:
            raise ReplayDatasetError(
                f"{name} frame count mismatch: expected {frame_count}, got {values.shape[0]}"
            )
        checked.append(values)

    if frame_count is None or frame_count == 0:
        raise ReplayDatasetError("demo contains no replay frames")
    return np.concatenate(checked, axis=1)


def append_optional_left_gripper(
    positions: np.ndarray,
    joint_names: list[str],
    left_gripper_joint_pos: np.ndarray | None,
) -> tuple[np.ndarray, list[str]]:
    if left_gripper_joint_pos is None:
        return positions, joint_names

    gripper = np.asarray(left_gripper_joint_pos, dtype=np.float32)
    if gripper.ndim != 2 or gripper.shape[1] != len(LEFT_GRIPPER_JOINT_NAMES):
        raise ReplayDatasetError(
            f"left_gripper_joint_pos must have shape (N,{len(LEFT_GRIPPER_JOINT_NAMES)}); got {tuple(gripper.shape)}"
        )
    if gripper.shape[0] != positions.shape[0]:
        raise ReplayDatasetError(
            f"left_gripper_joint_pos frame count mismatch: expected {positions.shape[0]}, got {gripper.shape[0]}"
        )
    return np.concatenate([positions, gripper], axis=1), joint_names + list(LEFT_GRIPPER_JOINT_NAMES)


def _read_joint_names_attr(dataset: h5py.Dataset) -> list[str] | None:
    raw = dataset.attrs.get("joint_names")
    if raw is None:
        return None
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    if isinstance(raw, str):
        decoded = json.loads(raw)
        return [str(name) for name in decoded]
    return [str(name.decode("utf-8") if isinstance(name, bytes) else name) for name in raw]


def _read_joint_dataset(obs: h5py.Group, key: str, expected_joint_names: Sequence[str]) -> np.ndarray:
    dataset = obs[key]
    values = np.asarray(dataset[:], dtype=np.float32)
    joint_names = _read_joint_names_attr(dataset)
    if joint_names is None:
        return values
    if len(joint_names) != values.shape[1]:
        raise ReplayDatasetError(
            f"{key} joint_names length mismatch: names={len(joint_names)} columns={values.shape[1]}"
        )
    index_by_name = {name: i for i, name in enumerate(joint_names)}
    missing = [name for name in expected_joint_names if name not in index_by_name]
    if missing:
        raise ReplayDatasetError(f"{key} is missing expected joint_names: {missing}")
    return values[:, [index_by_name[name] for name in expected_joint_names]]


def load_replay_trajectory(dataset_path: Path, demo_key: str) -> ReplayTrajectory:
    """Load and validate a demo trajectory from a pour mimic HDF5 file."""

    dataset_path = Path(dataset_path)
    if not dataset_path.is_file():
        raise ReplayDatasetError(f"dataset does not exist: {dataset_path}")

    demo_path = f"data/{demo_key}"
    with h5py.File(dataset_path, "r") as h5:
        if demo_path not in h5:
            raise ReplayDatasetError(f"missing demo group: {demo_path}")
        demo = h5[demo_path]
        obs = demo.get("obs")
        if obs is None:
            raise ReplayDatasetError(f"missing obs group: {demo_path}/obs")

        if "robot_replay_joint_pos" in obs:
            positions = _read_joint_dataset(
                obs, "robot_replay_joint_pos", EXPECTED_ROBOT_REPLAY_JOINT_NAMES
            )
            joint_names = EXPECTED_ROBOT_REPLAY_JOINT_NAMES.copy()
        else:
            required = ("right_arm_joint_pos", "right_hand_joint_pos", "left_arm_joint_pos")
            missing = [name for name in required if name not in obs]
            if missing:
                raise ReplayDatasetError(f"missing required obs datasets: {', '.join(missing)}")

            positions = build_dataset_joint_positions(
                _read_joint_dataset(obs, "right_arm_joint_pos", RIGHT_ARM_JOINT_NAMES),
                _read_joint_dataset(obs, "right_hand_joint_pos", RIGHT_HAND_JOINT_NAMES),
                _read_joint_dataset(obs, "left_arm_joint_pos", LEFT_ARM_JOINT_NAMES),
            )
            left_gripper = None
            joint_names = EXPECTED_JOINT_NAMES.copy()
            if "left_gripper_joint_pos" in obs:
                left_gripper = _read_joint_dataset(obs, "left_gripper_joint_pos", LEFT_GRIPPER_JOINT_NAMES)
            positions, joint_names = append_optional_left_gripper(positions, joint_names, left_gripper)
        timestamps_ns = demo["timestamps_ns"][:] if "timestamps_ns" in demo else None
        if timestamps_ns is not None and int(timestamps_ns.shape[0]) != int(positions.shape[0]):
            raise ReplayDatasetError(
                f"timestamps_ns frame count mismatch: expected {positions.shape[0]}, got {timestamps_ns.shape[0]}"
            )

    return ReplayTrajectory(positions=positions, joint_names=joint_names, timestamps_ns=timestamps_ns)


def find_missing_joints(expected: Sequence[str], available: Sequence[str]) -> list[str]:
    """Return expected joints that are absent from the USD articulation."""

    available_set = set(available)
    return [joint_name for joint_name in expected if joint_name not in available_set]


def _log(message: str) -> None:
    print(message, flush=True)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a real OpenArm + Tesollo pour HDF5 joint trajectory in IsaacSim."
    )
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET, help="Path to pour HDF5 dataset.")
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD, help="Path to OpenArm + Tesollo USD asset.")
    parser.add_argument("--demo-key", default="demo_0", help="Demo key under /data to replay.")
    parser.add_argument("--rate", type=float, default=100.0, help="Replay rate in Hz.")
    parser.add_argument("--loop", action="store_true", help="Loop the trajectory until the viewer exits.")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Stop after this many frames. Useful for headless smoke tests.",
    )
    parser.add_argument(
        "--robot-prim-path",
        default="/World/Robot",
        help="Prim path where the USD articulation should be spawned.",
    )

    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def _run_viewer(args: argparse.Namespace, trajectory: ReplayTrajectory) -> None:
    if args.rate <= 0.0:
        raise ValueError("--rate must be greater than zero")
    if not args.usd.is_file():
        raise FileNotFoundError(f"USD does not exist: {args.usd}")

    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    try:
        import torch

        import isaaclab.sim as sim_utils
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import Articulation, ArticulationCfg

        sim_dt = 1.0 / float(args.rate)
        _log("[INFO] Creating IsaacLab simulation context.")
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=sim_dt))
        sim.set_camera_view(eye=[1.9, -1.8, 1.3], target=[0.35, 0.0, 0.45])

        _log("[INFO] Spawning ground and light.")
        sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
        dome_cfg = sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0)
        dome_cfg.func("/World/Light", dome_cfg)

        _log(f"[INFO] Spawning robot articulation from USD: {args.usd}")
        robot_cfg = ArticulationCfg(
            prim_path=args.robot_prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(args.usd),
                activate_contact_sensors=False,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False),
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=[0.0, 0.0, 0.0], joint_vel={".*": 0.0}),
            actuators={
                "replay": ImplicitActuatorCfg(
                    joint_names_expr=[".*"],
                    stiffness=1000.0,
                    damping=100.0,
                )
            },
        )
        robot = Articulation(robot_cfg)

        _log("[INFO] Resetting simulation.")
        sim.reset()
        robot.reset()

        _log("[INFO] Resolving replay joints.")
        missing = find_missing_joints(trajectory.joint_names, robot.joint_names)
        if missing:
            available = ", ".join(robot.joint_names)
            raise RuntimeError(
                "USD articulation is missing replay joints: "
                f"{', '.join(missing)}\nAvailable joints: {available}"
            )
        joint_ids, resolved_joint_names = robot.find_joints(trajectory.joint_names, preserve_order=True)
        if list(resolved_joint_names) != trajectory.joint_names:
            raise RuntimeError(
                "resolved USD joint order does not match replay order: "
                f"{resolved_joint_names} != {trajectory.joint_names}"
            )

        frame_positions = torch.as_tensor(trajectory.positions, dtype=torch.float32, device=sim.device)
        joint_ids_tensor = torch.as_tensor(joint_ids, dtype=torch.long, device=sim.device)
        joint_pos = robot.data.default_joint_pos.clone()
        joint_vel = torch.zeros_like(robot.data.default_joint_vel)

        total_frames = trajectory.num_frames
        duration = total_frames / float(args.rate)
        _log(
            f"[INFO] Replaying {args.dataset}::{args.demo_key} "
            f"({total_frames} frames, {duration:.2f}s @ {args.rate:g} Hz)"
        )
        _log(f"[INFO] USD: {args.usd}")

        frame_index = 0
        played_frames = 0
        while simulation_app.is_running():
            joint_pos[:, joint_ids_tensor] = frame_positions[frame_index].unsqueeze(0)
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.set_joint_position_target(joint_pos)
            robot.write_data_to_sim()

            sim.step()
            robot.update(sim_dt)

            if played_frames % max(1, int(args.rate)) == 0:
                _log(
                    f"[INFO] frame={frame_index + 1}/{total_frames} "
                    f"time={frame_index / float(args.rate):.2f}s"
                )

            played_frames += 1
            if args.max_frames is not None and played_frames >= args.max_frames:
                _log(f"[INFO] Reached --max-frames={args.max_frames}; exiting.")
                break

            frame_index += 1
            if frame_index >= total_frames:
                if args.loop:
                    frame_index = 0
                    _log("[INFO] Looping replay.")
                else:
                    _log("[INFO] Replay complete.")
                    break
    finally:
        fast_headless_exit = bool(getattr(args, "headless", False) and args.max_frames is not None)
        simulation_app.close(wait_for_replicator=False, skip_cleanup=fast_headless_exit)


def main() -> None:
    args = _parse_args()
    trajectory = load_replay_trajectory(args.dataset, args.demo_key)
    _run_viewer(args, trajectory)


if __name__ == "__main__":
    main()
