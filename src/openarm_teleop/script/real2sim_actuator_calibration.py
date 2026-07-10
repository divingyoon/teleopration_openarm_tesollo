#!/usr/bin/env python3
"""Build Real2Sim actuator calibration files from command/response data.

The estimates produced here are intentionally conservative. They are meant to
seed Isaac Lab actuator parameters and randomization ranges, then be refined by
replaying the same trajectories in simulation.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import h5py
import numpy as np


DEFAULT_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class ActuatorGroupDefaults:
    stiffness: float
    damping: float
    effort_limit: float | None = None
    velocity_limit: float | None = None
    joint_friction: float = 0.0


@dataclass(frozen=True)
class CalibrationBuildConfig:
    robot_asset: str
    source_dataset: str
    groups: Mapping[str, Mapping[str, Any]]


@dataclass(frozen=True)
class Hdf5JointPair:
    command: np.ndarray
    measured: np.ndarray
    timestamps_ns: np.ndarray
    joint_names: list[str]


def _as_2d_float(values: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32)
    if array.ndim != 2:
        raise ValueError(f"{name} must be rank-2, got shape={array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains NaN or inf")
    return array


def _read_joint_names(dataset: h5py.Dataset, width: int) -> list[str]:
    raw = dataset.attrs.get("joint_names")
    if raw is None:
        return [f"joint_{i}" for i in range(width)]
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    if isinstance(raw, str):
        names = json.loads(raw)
    else:
        names = [item.decode("utf-8") if isinstance(item, bytes) else str(item) for item in raw]
    if len(names) != width:
        raise ValueError(f"joint_names length {len(names)} does not match dataset width {width}")
    return [str(name) for name in names]


def _dataset_at(demo: h5py.Group, relative_path: str) -> h5py.Dataset:
    node = demo
    for part in relative_path.split("/"):
        if not part:
            continue
        if part not in node:
            raise ValueError(f"missing dataset path: {relative_path}")
        node = node[part]
    if not isinstance(node, h5py.Dataset):
        raise ValueError(f"path is not a dataset: {relative_path}")
    return node


def load_hdf5_joint_pair(
    dataset_path: str | Path,
    demo_key: str,
    command_dataset: str,
    measured_dataset: str,
) -> Hdf5JointPair:
    dataset_path = Path(dataset_path)
    with h5py.File(dataset_path, "r") as h5:
        demo_path = f"data/{demo_key}"
        if demo_path not in h5:
            raise ValueError(f"missing demo group: {demo_path}")
        demo = h5[demo_path]
        cmd_ds = _dataset_at(demo, command_dataset)
        measured_ds = _dataset_at(demo, measured_dataset)
        command = _as_2d_float(cmd_ds[:], command_dataset)
        measured = _as_2d_float(measured_ds[:], measured_dataset)
        if command.shape != measured.shape:
            raise ValueError(
                f"command/measured shape mismatch: {command.shape} vs {measured.shape}"
            )
        if "timestamps_ns" in demo:
            timestamps_ns = np.asarray(demo["timestamps_ns"][:], dtype=np.int64)
        else:
            timestamps_ns = np.arange(command.shape[0], dtype=np.int64)
        if timestamps_ns.shape[0] != command.shape[0]:
            raise ValueError("timestamps_ns length does not match trajectory length")
        joint_names = _read_joint_names(cmd_ds, command.shape[1])
    return Hdf5JointPair(command=command, measured=measured, timestamps_ns=timestamps_ns, joint_names=joint_names)


def select_joint_pattern(pair: Hdf5JointPair, joint_name_regex: str | None) -> Hdf5JointPair:
    if not joint_name_regex:
        return pair
    pattern = re.compile(joint_name_regex)
    indices = [index for index, name in enumerate(pair.joint_names) if pattern.fullmatch(name)]
    if not indices:
        raise ValueError(f"joint_name_regex matched no joints: {joint_name_regex}")
    return Hdf5JointPair(
        command=pair.command[:, indices],
        measured=pair.measured[:, indices],
        timestamps_ns=pair.timestamps_ns,
        joint_names=[pair.joint_names[index] for index in indices],
    )


def _sample_period_sec(timestamps_ns: np.ndarray) -> float:
    if timestamps_ns.shape[0] < 2:
        return 0.0
    diffs = np.diff(timestamps_ns.astype(np.float64)) * 1e-9
    positive = diffs[diffs > 0.0]
    return float(np.median(positive)) if positive.size else 0.0


def _lag_steps(command: np.ndarray, measured: np.ndarray) -> int:
    cmd = command - float(np.mean(command))
    obs = measured - float(np.mean(measured))
    if float(np.linalg.norm(cmd)) < 1e-9 or float(np.linalg.norm(obs)) < 1e-9:
        return 0
    corr = np.correlate(obs, cmd, mode="full")
    return max(0, int(np.argmax(corr) - (command.shape[0] - 1)))


def _first_movement_threshold(command: np.ndarray, measured: np.ndarray) -> float:
    delta_cmd = np.abs(command - command[0])
    delta_measured = np.abs(measured - measured[0])
    threshold = max(0.01, float(np.max(delta_measured)) * 0.05)
    moved = np.flatnonzero(delta_measured >= threshold)
    if moved.size == 0:
        return float(np.max(delta_cmd))
    return float(delta_cmd[int(moved[0])])


def compute_joint_metrics(
    command: np.ndarray,
    measured: np.ndarray,
    timestamps_ns: np.ndarray,
    joint_names: Sequence[str],
) -> dict[str, dict[str, float]]:
    command = _as_2d_float(command, "command")
    measured = _as_2d_float(measured, "measured")
    if command.shape != measured.shape:
        raise ValueError("command and measured must have the same shape")
    if len(joint_names) != command.shape[1]:
        raise ValueError("joint_names length must match command width")

    dt = _sample_period_sec(timestamps_ns)
    metrics: dict[str, dict[str, float]] = {}
    for index, joint_name in enumerate(joint_names):
        cmd = command[:, index]
        obs = measured[:, index]
        err = cmd - obs
        if dt > 0.0 and obs.shape[0] > 1:
            max_velocity = float(np.max(np.abs(np.gradient(obs, dt))))
        else:
            max_velocity = 0.0
        lag = _lag_steps(cmd, obs)
        metrics[str(joint_name)] = {
            "rmse_rad": float(np.sqrt(np.mean(np.square(err)))),
            "mae_rad": float(np.mean(np.abs(err))),
            "steady_state_error_rad": np.float32(np.mean(err[-max(1, obs.shape[0] // 5) :])).item(),
            "lag_sec": round(float(lag * dt), 6),
            "deadband_command_rad": float(_first_movement_threshold(cmd, obs)),
            "max_velocity_rad_s": max_velocity,
        }
    return metrics


def _mean_metric(metrics: Mapping[str, Mapping[str, float]], key: str) -> float:
    values = [float(item[key]) for item in metrics.values()]
    return float(np.mean(values)) if values else 0.0


def estimate_group_calibration(
    group_name: str,
    command: np.ndarray,
    measured: np.ndarray,
    timestamps_ns: np.ndarray,
    joint_names: Sequence[str],
    defaults: ActuatorGroupDefaults,
) -> dict[str, Any]:
    metrics = compute_joint_metrics(command, measured, timestamps_ns, joint_names)
    mean_abs_ss = abs(_mean_metric(metrics, "steady_state_error_rad"))
    mean_lag = _mean_metric(metrics, "lag_sec")
    mean_deadband = _mean_metric(metrics, "deadband_command_rad")
    max_velocity = max(float(item["max_velocity_rad_s"]) for item in metrics.values())

    stiffness_scale = float(np.clip(1.0 + mean_abs_ss * 2.0 + mean_deadband, 0.5, 3.0))
    damping_scale = float(np.clip(1.0 + mean_lag * 20.0, 0.5, 3.0))
    friction = max(float(defaults.joint_friction), float(mean_deadband * max(defaults.stiffness, 1.0) * 0.05))
    dt = _sample_period_sec(timestamps_ns)
    delay_steps = int(round(mean_lag / dt)) if dt > 0.0 else 0

    result: dict[str, Any] = {
        "stiffness": float(defaults.stiffness * stiffness_scale),
        "damping": float(defaults.damping * damping_scale),
        "joint_friction": friction,
        "delay_steps": max(0, delay_steps),
        "fit_error": {
            "rmse_rad": _mean_metric(metrics, "rmse_rad"),
            "mae_rad": _mean_metric(metrics, "mae_rad"),
            "steady_state_error_rad": _mean_metric(metrics, "steady_state_error_rad"),
            "lag_sec": mean_lag,
        },
        "joint_metrics": metrics,
    }
    if defaults.effort_limit is not None:
        result["effort_limit"] = float(defaults.effort_limit)
    if defaults.velocity_limit is not None:
        result["velocity_limit"] = float(max(defaults.velocity_limit, max_velocity))
    return result


def write_calibration_json(output_path: str | Path, config: CalibrationBuildConfig) -> None:
    payload = {
        "schema_version": DEFAULT_SCHEMA_VERSION,
        "robot_asset": config.robot_asset,
        "source_dataset": config.source_dataset,
        "groups": dict(config.groups),
    }
    Path(output_path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _parse_defaults(raw: str) -> ActuatorGroupDefaults:
    parts = [float(item) for item in raw.split(",")]
    if len(parts) not in {2, 4, 5}:
        raise argparse.ArgumentTypeError(
            "defaults must be stiffness,damping[,effort_limit,velocity_limit[,joint_friction]]"
        )
    return ActuatorGroupDefaults(
        stiffness=parts[0],
        damping=parts[1],
        effort_limit=parts[2] if len(parts) >= 4 else None,
        velocity_limit=parts[3] if len(parts) >= 4 else None,
        joint_friction=parts[4] if len(parts) == 5 else 0.0,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate Real2Sim actuator calibration from HDF5 data.")
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--demo-key", default="demo_0")
    parser.add_argument("--group-name", required=True)
    parser.add_argument("--command-dataset", required=True, help="Dataset path relative to data/<demo>, e.g. obs/right_hand_reference_joint_pos")
    parser.add_argument("--measured-dataset", required=True, help="Dataset path relative to data/<demo>, e.g. obs/right_hand_joint_pos")
    parser.add_argument(
        "--joint-name-regex",
        default=None,
        help="Optional full-match regex used to select columns by joint_names attr.",
    )
    parser.add_argument("--defaults", required=True, type=_parse_defaults)
    parser.add_argument("--robot-asset", default="openarm_tesollo_sensor")
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    pair = load_hdf5_joint_pair(args.dataset, args.demo_key, args.command_dataset, args.measured_dataset)
    pair = select_joint_pattern(pair, args.joint_name_regex)
    group = estimate_group_calibration(
        group_name=args.group_name,
        command=pair.command,
        measured=pair.measured,
        timestamps_ns=pair.timestamps_ns,
        joint_names=pair.joint_names,
        defaults=args.defaults,
    )
    write_calibration_json(
        args.output,
        CalibrationBuildConfig(
            robot_asset=args.robot_asset,
            source_dataset=str(args.dataset),
            groups={args.group_name: group},
        ),
    )
    print(f"[INFO] wrote calibration: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
