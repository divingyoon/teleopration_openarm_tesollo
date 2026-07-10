"""Real2Sim identification bag → HDF5 변환의 순수 코어.

ROS/rosbag 의존이 없다. db3 읽기는 db3_to_identification_hdf5.py가 담당하고,
여기서는 이미 (timestamp, values) 로 풀린 샘플만 다룬다. 그래야 테스트할 수 있다.

출력 schema는 hdgp/scripts/r2s_autotune/load_real_track.py가 기대하는 것과 같다.

    data/demo_0/timestamps_ns
    data/demo_0/obs/q_cmd    [T, J]  attr joint_names (JSON)
    data/demo_0/obs/q_real   [T, J]  attr joint_names (JSON)
    data/demo_0/obs/dq_real  [T, J]  attr joint_names (JSON)

joint_names는 실물이 쓰는 legacy 이름(rj_dg_*)을 그대로 둔다.
canonical 정규화는 load_real_track이 manifest로 수행한다.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


class IdentificationError(ValueError):
    """수집된 bag이 식별에 쓸 수 없는 상태. 조용히 넘어가면 MSE가 무의미해진다."""


@dataclass(frozen=True)
class NamedSample:
    """한 시점의 관절 값. names는 매 샘플마다 같다고 가정하지 않는다."""

    timestamp_ns: int
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...] | None = None


@dataclass(frozen=True)
class IdentificationTrack:
    timestamps_ns: np.ndarray  # [T]
    q_cmd: np.ndarray  # [T, J]
    q_real: np.ndarray  # [T, J]
    dq_real: np.ndarray  # [T, J]
    joint_names: tuple[str, ...]


def _as_lookup(sample: NamedSample) -> dict[str, tuple[float, float]]:
    velocities = sample.velocities or (0.0,) * len(sample.positions)
    if len(sample.names) != len(sample.positions):
        raise IdentificationError(
            f"names({len(sample.names)}) and positions({len(sample.positions)}) length differ"
        )
    return {n: (p, v) for n, p, v in zip(sample.names, sample.positions, velocities)}


def _reorder(sample: NamedSample, joint_names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
    lookup = _as_lookup(sample)
    missing = [n for n in joint_names if n not in lookup]
    if missing:
        raise IdentificationError(f"sample is missing joints: {missing}")
    values = np.array([lookup[n] for n in joint_names], dtype=np.float64)
    return values[:, 0], values[:, 1]


def _time_grid(
    command: Sequence[NamedSample],
    measured: Sequence[NamedSample],
    dt: float,
) -> np.ndarray:
    """두 스트림이 모두 살아 있는 구간에만 균일 격자를 놓는다.

    한쪽만 있는 구간을 포함하면 nearest 보간이 경계값을 늘려 붙여 가짜 정상상태를 만든다.
    """
    start = max(command[0].timestamp_ns, measured[0].timestamp_ns)
    end = min(command[-1].timestamp_ns, measured[-1].timestamp_ns)
    if end <= start:
        raise IdentificationError("command and measured streams do not overlap in time")

    step_ns = int(round(dt * 1e9))
    steps = int((end - start) // step_ns) + 1
    if steps < 2:
        raise IdentificationError(f"overlap too short for dt={dt}s")
    return start + np.arange(steps, dtype=np.int64) * step_ns


def _resample_nearest(samples: Sequence[NamedSample], grid: np.ndarray) -> list[NamedSample]:
    source = np.array([s.timestamp_ns for s in samples], dtype=np.int64)
    index = np.searchsorted(source, grid, side="left")
    index = np.clip(index, 1, len(samples) - 1)
    left, right = index - 1, index
    pick = np.where(grid - source[left] <= source[right] - grid, left, right)
    return [samples[int(i)] for i in pick]


def _finite_diff(q: np.ndarray, dt: float) -> np.ndarray:
    if q.shape[0] < 2:
        return np.zeros_like(q)
    return np.gradient(q, dt, axis=0)


def build_identification_track(
    command: Sequence[NamedSample],
    measured: Sequence[NamedSample],
    dt: float,
    joint_names: Sequence[str] | None = None,
) -> IdentificationTrack:
    """명령/측정 스트림을 공통 dt 격자로 정렬해 track을 만든다.

    joint_names를 주지 않으면 첫 command 샘플의 dof_names 순서를 따른다
    (MultiDOFCommand는 자기서술적이다).
    """
    if not command or not measured:
        raise IdentificationError("command or measured stream is empty")
    if dt <= 0.0:
        raise IdentificationError("dt must be positive")

    names = tuple(joint_names) if joint_names else tuple(command[0].names)
    if not names:
        raise IdentificationError("command samples carry no joint names")

    grid = _time_grid(command, measured, dt)
    cmd_samples = _resample_nearest(command, grid)
    meas_samples = _resample_nearest(measured, grid)

    q_cmd = np.stack([_reorder(s, names)[0] for s in cmd_samples])
    reordered = [_reorder(s, names) for s in meas_samples]
    q_real = np.stack([r[0] for r in reordered])
    dq_real = np.stack([r[1] for r in reordered])

    # JointState.velocity가 비어 있으면 0으로 채워져 온다. 그때는 위치를 미분한다.
    if not np.any(dq_real):
        dq_real = _finite_diff(q_real, dt)

    return IdentificationTrack(
        timestamps_ns=grid,
        q_cmd=q_cmd,
        q_real=q_real,
        dq_real=dq_real,
        joint_names=names,
    )


def assert_identifiable(track: IdentificationTrack, min_amplitude_rad: float = 0.05) -> None:
    """명령이 실제로 움직였고 측정이 명령을 그대로 되비추지 않는지 확인한다.

    가이드 §11.2의 두 실패 모드를 수집 직후에 잡는다.
    """
    amplitude = np.ptp(track.q_cmd, axis=0)
    if float(amplitude.max()) < min_amplitude_rad:
        raise IdentificationError(
            f"command barely moves (max amplitude {amplitude.max():.4f} rad). "
            "excitation 노드가 실제로 발행됐는지 확인하라."
        )
    if np.allclose(track.q_cmd, track.q_real):
        raise IdentificationError(
            "command equals measured. 같은 토픽을 두 번 읽고 있을 수 있다."
        )


def write_identification_hdf5(
    path: str | Path,
    track: IdentificationTrack,
    attrs: dict[str, str] | None = None,
) -> None:
    import h5py

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    names_json = json.dumps(list(track.joint_names))

    with h5py.File(path, "w") as handle:
        demo = handle.create_group("data/demo_0")
        demo.attrs["num_samples"] = int(track.q_cmd.shape[0])
        demo.create_dataset("timestamps_ns", data=track.timestamps_ns)
        obs = demo.create_group("obs")
        for key, array in (
            ("q_cmd", track.q_cmd),
            ("q_real", track.q_real),
            ("dq_real", track.dq_real),
        ):
            dataset = obs.create_dataset(key, data=array.astype(np.float32), compression="gzip")
            dataset.attrs["joint_names"] = names_json
        for key, value in (attrs or {}).items():
            handle["data"].attrs[key] = value
