#!/usr/bin/env python3
"""Convert ROS2 bag DB3 demos to pour_v1_mimic HDF5 demos."""

from __future__ import annotations

import argparse
import json
import sqlite3
from dataclasses import dataclass
from collections import deque
from pathlib import Path
from typing import Callable, Any

import h5py
import numpy as np

from openarm_fk import fk_trajectory, pose_to_action_delta
from pour_v1_mimic_contract import (
    ACTION_DIM,
    ACTOR_OBSERVATION_DIM,
    OBSERVATION_DIM_BY_KEY,
    OPTIONAL_TOPICS,
    REQUIRED_TOPICS,
    RIGHT_HAND_CURL_SLICE,
    TASK_ID_MIMIC,
    TOPIC_NOMINAL_HZ,
    validate_action_tensor,
)

# Phase-3 default static cup poses in world frame.
# TODO(phase4): replace with TF-derived cup poses when bag carries reliable object tracking.
_DEFAULT_SOURCE_CUP_POSE_W = np.array(
    [
        [1.0, 0.0, 0.0, 0.35],
        [0.0, 1.0, 0.0, -0.10],
        [0.0, 0.0, 1.0, 0.05],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)
_DEFAULT_TARGET_CUP_POSE_W = np.array(
    [
        [1.0, 0.0, 0.0, 0.35],
        [0.0, 1.0, 0.0, 0.10],
        [0.0, 0.0, 1.0, 0.05],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)


@dataclass
class TopicSample:
    timestamp_ns: int
    value: Any


@dataclass
class ObjectPoseConfig:
    mode: str
    source_cup_pose_w: np.ndarray
    target_cup_pose_w: np.ndarray
    tf_topic: str
    source_cup_frame: str
    target_cup_frame: str
    tf_reference_frame: str


@dataclass
class SubtaskSignalConfig:
    grasp_curl_threshold: float
    lift_z_margin: float
    align_xy_threshold: float


@dataclass
class TfEdge:
    parent_frame: str
    child_frame: str
    transform_pc: np.ndarray


def _build_topic_deserializer(topic_to_type: dict[str, str]) -> dict[str, Callable[[bytes], Any]]:
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS deserialization dependencies missing. "
            "Run this tool in a ROS2 environment with rclpy installed."
        ) from exc

    def _joint_state_pos_vel(msg) -> np.ndarray:
        pos = np.asarray(msg.position, dtype=np.float32)
        vel = np.asarray(msg.velocity, dtype=np.float32)
        if vel.size == 0:
            vel = np.zeros_like(pos)
        elif vel.shape[0] != pos.shape[0]:
            fixed = np.zeros_like(pos)
            keep = min(pos.shape[0], vel.shape[0])
            fixed[:keep] = vel[:keep]
            vel = fixed
        return np.concatenate([pos, vel], axis=0)

    def _multi_dof_values(msg) -> np.ndarray:
        return np.asarray(msg.values, dtype=np.float32)

    def _float_array(msg) -> np.ndarray:
        return np.asarray(msg.data, dtype=np.float32)

    def _tf_message(msg) -> list[TfEdge]:
        out: list[TfEdge] = []
        for t in msg.transforms:
            parent = _normalize_frame_id(t.header.frame_id)
            child = _normalize_frame_id(t.child_frame_id)
            if not parent or not child:
                continue
            xyz = np.array(
                [
                    float(t.transform.translation.x),
                    float(t.transform.translation.y),
                    float(t.transform.translation.z),
                ],
                dtype=np.float64,
            )
            quat_xyzw = np.array(
                [
                    float(t.transform.rotation.x),
                    float(t.transform.rotation.y),
                    float(t.transform.rotation.z),
                    float(t.transform.rotation.w),
                ],
                dtype=np.float64,
            )
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = _quat_xyzw_to_rotmat(quat_xyzw)
            T[:3, 3] = xyz
            out.append(TfEdge(parent_frame=parent, child_frame=child, transform_pc=T.astype(np.float32)))
        return out

    extractors: dict[str, Callable[[object], Any]] = {
        "/openarm/left/joint_states": _joint_state_pos_vel,
        "/openarm/right/joint_states": _joint_state_pos_vel,
        "/dg5f_right/rj_dg_pospid/reference": _multi_dof_values,
        "/dg5f_right/joint_states": _joint_state_pos_vel,
        "/tesollo/right/sensor": _float_array,
        "/tf": _tf_message,
        "/tf_static": _tf_message,
    }
    deserializers: dict[str, Callable[[bytes], Any]] = {}
    for topic_name, topic_type in topic_to_type.items():
        if topic_name not in extractors:
            continue
        msg_cls = get_message(topic_type)
        extractor = extractors[topic_name]

        def _decode(raw_data: bytes, msg_cls=msg_cls, extractor=extractor) -> Any:
            msg = deserialize_message(raw_data, msg_cls)
            return extractor(msg)

        deserializers[topic_name] = _decode
    return deserializers


def load_samples_from_bag_dir(bag_dir: Path) -> dict[str, list[TopicSample]]:
    db3_files = sorted(bag_dir.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"no .db3 file found under: {bag_dir}")

    out: dict[str, list[TopicSample]] = {k: [] for k in REQUIRED_TOPICS + OPTIONAL_TOPICS}
    for db3_file in db3_files:
        conn = sqlite3.connect(str(db3_file))
        cur = conn.cursor()
        cur.execute("SELECT id, name, type FROM topics")
        topic_rows = cur.fetchall()
        id_to_topic = {topic_id: (name, msg_type) for topic_id, name, msg_type in topic_rows}
        topic_to_type = {name: msg_type for _, name, msg_type in topic_rows}
        deserializers = _build_topic_deserializer(topic_to_type)

        cur.execute("SELECT timestamp, topic_id, data FROM messages ORDER BY timestamp ASC")
        for timestamp_ns, topic_id, raw_data in cur.fetchall():
            if topic_id not in id_to_topic:
                continue
            topic_name = id_to_topic[topic_id][0]
            if topic_name not in out or topic_name not in deserializers:
                continue
            try:
                value = deserializers[topic_name](raw_data)
                out[topic_name].append(TopicSample(timestamp_ns=int(timestamp_ns), value=value))
            except Exception:
                continue
        conn.close()

    for topic_name in REQUIRED_TOPICS:
        if len(out[topic_name]) == 0:
            raise ValueError(f"required topic is empty in bag: {topic_name}")
    return out


def run_quality_gate(
    topic_samples: dict[str, list[TopicSample]],
    max_drop_rate: float,
    max_gap_sec: float,
) -> dict:
    required_counts = {topic: len(topic_samples[topic]) for topic in REQUIRED_TOPICS}

    # Compute recording duration from the timestamp span across all required topics.
    all_starts = [topic_samples[t][0].timestamp_ns for t in REQUIRED_TOPICS if topic_samples[t]]
    all_ends = [topic_samples[t][-1].timestamp_ns for t in REQUIRED_TOPICS if topic_samples[t]]
    duration_sec = (max(all_ends) - min(all_starts)) / 1e9 if all_starts and all_ends else 0.0

    # Evaluate each required topic against its own nominal rate × duration so
    # slow topics (e.g. dg5f reference at 100 Hz) are not penalised against
    # fast topics (arm joints at 1000 Hz).
    drop_by_topic: dict[str, float] = {}
    expected_by_topic: dict[str, float] = {}
    for topic, count in required_counts.items():
        nominal_hz = TOPIC_NOMINAL_HZ.get(topic)
        if nominal_hz is not None and duration_sec > 0:
            expected = max(1.0, nominal_hz * duration_sec)
        else:
            expected = float(max(required_counts.values()))
        expected_by_topic[topic] = expected
        drop = 1.0 - float(count) / expected
        drop_by_topic[topic] = drop
        if drop > max_drop_rate:
            raise ValueError(
                f"quality gate failed: topic={topic} drop_rate={drop:.3f} "
                f"> max_drop_rate={max_drop_rate:.3f}"
            )

    # Evaluate timeline gap using right arm topic as reference.
    ts = np.asarray([s.timestamp_ns for s in topic_samples["/openarm/right/joint_states"]], dtype=np.int64)
    if ts.size > 1:
        gap_sec = np.diff(ts).astype(np.float64) / 1e9
        max_gap = float(np.max(gap_sec))
    else:
        max_gap = float("inf")
    if max_gap > max_gap_sec:
        raise ValueError(
            f"quality gate failed: max_gap_sec={max_gap:.6f} > allowed={max_gap_sec:.6f}"
        )

    return {
        "required_counts": required_counts,
        "expected_by_topic": expected_by_topic,
        "drop_by_topic": drop_by_topic,
        "max_gap_sec": max_gap,
        "recording_duration_sec": duration_sec,
    }


def _resample_nearest(samples: list[TopicSample], query_ts: np.ndarray) -> np.ndarray:
    src_ts = np.asarray([s.timestamp_ns for s in samples], dtype=np.int64)
    src_vals = [s.value for s in samples]
    if len(src_vals) == 0:
        raise ValueError("cannot resample empty sample list")

    src_dim = int(src_vals[0].shape[0])
    out = np.zeros((query_ts.shape[0], src_dim), dtype=np.float32)
    idx = np.searchsorted(src_ts, query_ts, side="left")
    idx = np.clip(idx, 0, src_ts.shape[0] - 1)

    for i, (qi, ii) in enumerate(zip(query_ts, idx, strict=True)):
        candidate = ii
        if ii > 0 and abs(qi - src_ts[ii - 1]) < abs(src_ts[ii] - qi):
            candidate = ii - 1
        out[i] = src_vals[candidate]
    return out


def _hand_20d_to_curl_5d(hand20: np.ndarray) -> np.ndarray:
    if hand20.shape[-1] != 20:
        raise ValueError(f"expected hand20 last dim=20, got {hand20.shape[-1]}")
    return hand20.reshape(hand20.shape[0], 5, 4).mean(axis=2)


def _sensor30_to_tip_force_norm(sensor30: np.ndarray, force_max: float = 10.0) -> np.ndarray:
    if sensor30.shape[-1] != 30:
        raise ValueError(f"expected sensor30 last dim=30, got {sensor30.shape[-1]}")
    # Layout: [fx, fy, fz, tx, ty, tz] x 5 fingers.
    wrench = sensor30.reshape(sensor30.shape[0], 5, 6)
    force = wrench[:, :, :3]
    norm = np.linalg.norm(force, axis=2)
    return np.clip(norm / float(force_max), 0.0, 1.0).astype(np.float32)


def _delta(x: np.ndarray) -> np.ndarray:
    if x.shape[0] == 0:
        return np.zeros_like(x)
    d = np.zeros_like(x)
    d[1:] = x[1:] - x[:-1]
    return d


def _right_eef_taskspace_delta(right_js: np.ndarray) -> np.ndarray:
    """Compute 6D task-space delta from right arm joint trajectory via FK.

    action[t] = pose_to_action_delta(T[t], T[t+1])  for t = 0..T-2
    action[T-1] = 0  (zero delta at last step)

    Matches pour_mimic_math.target_pose_to_action(action[0:6]):
      [0:3] world-frame XYZ translation delta (m)
      [3:6] world-frame axis-angle rotation delta (rad, left multiplication)
    """
    T_traj = fk_trajectory(right_js.astype(np.float64), arm="right")
    n = right_js.shape[0]
    deltas = np.zeros((n, 6), dtype=np.float64)
    for t in range(n - 1):
        deltas[t] = pose_to_action_delta(T_traj[t], T_traj[t + 1])
    return deltas.astype(np.float32)


def _previous_actions(actions: np.ndarray) -> np.ndarray:
    out = np.zeros_like(actions, dtype=np.float32)
    if actions.shape[0] > 1:
        out[1:] = actions[:-1]
    return out


def _normalize_dim(x: np.ndarray, dim: int) -> np.ndarray:
    out = np.zeros((x.shape[0], dim), dtype=np.float32)
    keep = min(dim, x.shape[1])
    out[:, :keep] = x[:, :keep]
    return out


def _shift_target_pose_seq(eef_pose_seq: np.ndarray) -> np.ndarray:
    if eef_pose_seq.shape[0] == 0:
        return eef_pose_seq.copy()
    return np.concatenate([eef_pose_seq[1:], eef_pose_seq[-1:]], axis=0)


def _parse_pose_matrix_arg(raw: str, arg_name: str) -> np.ndarray:
    values = [float(v.strip()) for v in raw.split(",") if v.strip()]
    if len(values) != 16:
        raise ValueError(
            f"{arg_name} must contain 16 comma-separated numbers (row-major 4x4), got={len(values)}"
        )
    mat = np.asarray(values, dtype=np.float32).reshape(4, 4)
    return mat


def _normalize_frame_id(frame_id: str) -> str:
    return str(frame_id).strip().lstrip("/")


def _quat_xyzw_to_rotmat(quat_xyzw: np.ndarray) -> np.ndarray:
    q = quat_xyzw.astype(np.float64)
    n = float(np.linalg.norm(q))
    if n < 1e-10:
        return np.eye(3, dtype=np.float64)
    q = q / n
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _invert_pose(T: np.ndarray) -> np.ndarray:
    T_inv = np.eye(4, dtype=np.float32)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def _resolve_tf_path(
    edge_map: dict[tuple[str, str], np.ndarray],
    source_frame: str,
    target_frame: str,
) -> np.ndarray | None:
    if source_frame == target_frame:
        return np.eye(4, dtype=np.float32)
    if not edge_map:
        return None

    neighbors: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (parent, child), T_pc in edge_map.items():
        neighbors.setdefault(parent, []).append((child, T_pc))
        neighbors.setdefault(child, []).append((parent, _invert_pose(T_pc)))

    queue: deque[tuple[str, np.ndarray]] = deque([(source_frame, np.eye(4, dtype=np.float32))])
    visited = {source_frame}
    while queue:
        frame, T_source_frame = queue.popleft()
        for nxt, T_frame_nxt in neighbors.get(frame, []):
            if nxt in visited:
                continue
            T_source_nxt = T_source_frame @ T_frame_nxt
            if nxt == target_frame:
                return T_source_nxt
            visited.add(nxt)
            queue.append((nxt, T_source_nxt))
    return None


def _build_object_pose_sequences(
    topic_samples: dict[str, list[TopicSample]],
    query_ts: np.ndarray,
    num_steps: int,
    object_pose_cfg: ObjectPoseConfig,
) -> dict[str, np.ndarray]:
    if object_pose_cfg.mode == "static":
        return {
            "source_cup": np.tile(object_pose_cfg.source_cup_pose_w[None, :, :], (num_steps, 1, 1)),
            "target_cup": np.tile(object_pose_cfg.target_cup_pose_w[None, :, :], (num_steps, 1, 1)),
        }

    if object_pose_cfg.mode == "tf":
        source_default = object_pose_cfg.source_cup_pose_w.astype(np.float32)
        target_default = object_pose_cfg.target_cup_pose_w.astype(np.float32)
        source_seq = np.tile(source_default[None, :, :], (num_steps, 1, 1))
        target_seq = np.tile(target_default[None, :, :], (num_steps, 1, 1))

        dynamic_samples = topic_samples.get(object_pose_cfg.tf_topic, [])
        static_samples = topic_samples.get("/tf_static", [])

        dynamic_events: list[tuple[int, TfEdge]] = []
        for s in dynamic_samples:
            if not isinstance(s.value, list):
                continue
            for edge in s.value:
                if isinstance(edge, TfEdge):
                    dynamic_events.append((s.timestamp_ns, edge))
        dynamic_events.sort(key=lambda x: x[0])

        edge_map: dict[tuple[str, str], np.ndarray] = {}
        for s in static_samples:
            if not isinstance(s.value, list):
                continue
            for edge in s.value:
                if isinstance(edge, TfEdge):
                    edge_map[(edge.parent_frame, edge.child_frame)] = edge.transform_pc

        source_frame = _normalize_frame_id(object_pose_cfg.source_cup_frame)
        target_frame = _normalize_frame_id(object_pose_cfg.target_cup_frame)
        ref_frame = _normalize_frame_id(object_pose_cfg.tf_reference_frame)

        dynamic_idx = 0
        fallback_source = 0
        fallback_target = 0
        for i, ts in enumerate(query_ts):
            while dynamic_idx < len(dynamic_events) and dynamic_events[dynamic_idx][0] <= int(ts):
                _, edge = dynamic_events[dynamic_idx]
                edge_map[(edge.parent_frame, edge.child_frame)] = edge.transform_pc
                dynamic_idx += 1

            T_ref_source = _resolve_tf_path(edge_map, ref_frame, source_frame)
            if T_ref_source is not None:
                source_seq[i] = T_ref_source
            else:
                fallback_source += 1

            T_ref_target = _resolve_tf_path(edge_map, ref_frame, target_frame)
            if T_ref_target is not None:
                target_seq[i] = T_ref_target
            else:
                fallback_target += 1

        print(
            "[INFO] object_pose_mode=tf "
            f"source_fallback_steps={fallback_source}/{num_steps} "
            f"target_fallback_steps={fallback_target}/{num_steps} "
            f"tf_topic={object_pose_cfg.tf_topic} ref={ref_frame}"
        )
        return {
            "source_cup": source_seq,
            "target_cup": target_seq,
        }

    raise ValueError(f"unsupported object pose mode: {object_pose_cfg.mode}")


def _first_true_index(mask: np.ndarray, start_idx: int = 0) -> int | None:
    start_idx = max(0, int(start_idx))
    if start_idx >= mask.shape[0]:
        return None
    indices = np.flatnonzero(mask[start_idx:])
    if indices.size == 0:
        return None
    return start_idx + int(indices[0])


def _make_cumulative_signal(num_steps: int, start_idx: int) -> np.ndarray:
    out = np.zeros((num_steps,), dtype=bool)
    if num_steps == 0:
        return out
    idx = int(np.clip(start_idx, 0, num_steps - 1))
    out[idx:] = True
    return out


def _compute_subtask_signals(
    right_eef_pose: np.ndarray,
    right_hand_curl: np.ndarray,
    target_cup_pose: np.ndarray,
    cfg: SubtaskSignalConfig,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], dict[str, float]]:
    num_steps = int(right_eef_pose.shape[0])

    curl_avg = right_hand_curl.astype(np.float32).mean(axis=1)
    eef_z = right_eef_pose[:, 2, 3].astype(np.float32)
    eef_xy = right_eef_pose[:, :2, 3].astype(np.float32)
    target_xy = target_cup_pose[:, :2, 3].astype(np.float32)
    dist_xy = np.linalg.norm(eef_xy - target_xy, axis=1)

    table_z = float(np.median(target_cup_pose[:, 2, 3].astype(np.float32)))
    lift_threshold_z = float(table_z + cfg.lift_z_margin)

    grasp_idx = _first_true_index(curl_avg >= cfg.grasp_curl_threshold, 0)
    if grasp_idx is None:
        grasp_idx = num_steps - 1

    lift_idx = _first_true_index(eef_z >= lift_threshold_z, grasp_idx)
    if lift_idx is None:
        lift_idx = num_steps - 1

    align_idx = _first_true_index(dist_xy <= cfg.align_xy_threshold, lift_idx)
    if align_idx is None:
        align_idx = num_steps - 1

    pour_idx = num_steps - 1

    term = {
        "grasp_done": _make_cumulative_signal(num_steps, grasp_idx),
        "lift_done": _make_cumulative_signal(num_steps, lift_idx),
        "align_done": _make_cumulative_signal(num_steps, align_idx),
        "pour_done": _make_cumulative_signal(num_steps, pour_idx),
    }
    start = {
        "grasp_start": _make_cumulative_signal(num_steps, 0),
        "lift_start": _make_cumulative_signal(num_steps, grasp_idx),
        "align_start": _make_cumulative_signal(num_steps, lift_idx),
        "pour_start": _make_cumulative_signal(num_steps, align_idx),
    }
    summary = {
        "grasp_idx": float(grasp_idx),
        "lift_idx": float(lift_idx),
        "align_idx": float(align_idx),
        "pour_idx": float(pour_idx),
        "table_z": table_z,
        "lift_threshold_z": lift_threshold_z,
    }
    return term, start, summary


def _build_datagen_info(
    right_js: np.ndarray,
    left_js: np.ndarray,
    right_hand_curl: np.ndarray,
    topic_samples: dict[str, list[TopicSample]],
    query_ts: np.ndarray,
    object_pose_cfg: ObjectPoseConfig,
    subtask_signal_cfg: SubtaskSignalConfig,
) -> dict:
    right_eef_pose = fk_trajectory(right_js.astype(np.float64), arm="right").astype(np.float32)
    left_eef_pose = fk_trajectory(left_js.astype(np.float64), arm="left").astype(np.float32)

    num_steps = right_js.shape[0]
    object_poses = _build_object_pose_sequences(
        topic_samples=topic_samples,
        query_ts=query_ts,
        num_steps=num_steps,
        object_pose_cfg=object_pose_cfg,
    )
    term_signals, start_signals, signal_summary = _compute_subtask_signals(
        right_eef_pose=right_eef_pose,
        right_hand_curl=right_hand_curl,
        target_cup_pose=object_poses["target_cup"],
        cfg=subtask_signal_cfg,
    )
    print(
        "[INFO] subtask_term_indices "
        f"grasp={int(signal_summary['grasp_idx'])} "
        f"lift={int(signal_summary['lift_idx'])} "
        f"align={int(signal_summary['align_idx'])} "
        f"pour={int(signal_summary['pour_idx'])}"
    )

    return {
        "eef_pose": {
            "right": right_eef_pose,
            "left": left_eef_pose,
        },
        "target_eef_pose": {
            "right": _shift_target_pose_seq(right_eef_pose),
            "left": _shift_target_pose_seq(left_eef_pose),
        },
        "object_pose": object_poses,
        "subtask_term_signals": term_signals,
        "subtask_start_signals": start_signals,
    }


def build_demo(
    topic_samples: dict[str, list[TopicSample]],
    target_hz: int,
    object_pose_cfg: ObjectPoseConfig | None = None,
    subtask_signal_cfg: SubtaskSignalConfig | None = None,
) -> dict:
    if target_hz <= 0:
        raise ValueError("target_hz must be > 0")
    if object_pose_cfg is None:
        object_pose_cfg = ObjectPoseConfig(
            mode="static",
            source_cup_pose_w=_DEFAULT_SOURCE_CUP_POSE_W.copy(),
            target_cup_pose_w=_DEFAULT_TARGET_CUP_POSE_W.copy(),
            tf_topic="/tf",
            source_cup_frame="source_cup",
            target_cup_frame="target_cup",
            tf_reference_frame="world",
        )
    if subtask_signal_cfg is None:
        subtask_signal_cfg = SubtaskSignalConfig(
            grasp_curl_threshold=0.55,
            lift_z_margin=0.06,
            align_xy_threshold=0.08,
        )

    all_start = max(topic_samples[t][0].timestamp_ns for t in REQUIRED_TOPICS)
    all_end = min(topic_samples[t][-1].timestamp_ns for t in REQUIRED_TOPICS)
    if all_end <= all_start:
        raise ValueError("non-overlapping required topic timelines")

    step_ns = int(1e9 / target_hz)
    grid = np.arange(all_start, all_end + 1, step_ns, dtype=np.int64)
    if grid.shape[0] < 2:
        raise ValueError("not enough synchronized samples after resampling")

    left_js = _normalize_dim(
        _resample_nearest(topic_samples["/openarm/left/joint_states"], grid), dim=14
    )
    right_js = _normalize_dim(
        _resample_nearest(topic_samples["/openarm/right/joint_states"], grid), dim=14
    )
    hand_ref = _normalize_dim(
        _resample_nearest(topic_samples["/dg5f_right/rj_dg_pospid/reference"], grid), dim=20
    )

    if topic_samples["/dg5f_right/joint_states"]:
        hand_state = _normalize_dim(_resample_nearest(topic_samples["/dg5f_right/joint_states"], grid), dim=40)
    else:
        hand_state = np.concatenate([hand_ref.copy(), np.zeros_like(hand_ref)], axis=1).astype(np.float32)
    if topic_samples["/tesollo/right/sensor"]:
        sensor = _normalize_dim(_resample_nearest(topic_samples["/tesollo/right/sensor"], grid), dim=30)
    else:
        sensor = np.zeros((grid.shape[0], 30), dtype=np.float32)

    right_arm_pos = right_js[:, :7]
    right_arm_vel = right_js[:, 7:14]
    left_arm_pos = left_js[:, :7]
    left_arm_vel = left_js[:, 7:14]
    right_hand_pos = hand_state[:, :20]
    right_hand_vel = hand_state[:, 20:40]

    right_palm_delta = _right_eef_taskspace_delta(right_arm_pos)
    right_curl = _hand_20d_to_curl_5d(hand_ref)
    left_delta = _delta(left_arm_pos)

    actions = np.concatenate([right_palm_delta, right_curl, left_delta], axis=1).astype(np.float32)
    validate_action_tensor(actions)
    prev_actions = _previous_actions(actions)
    tip_force_norm = _sensor30_to_tip_force_norm(sensor)
    right_joint_pos = np.concatenate([right_arm_pos, right_hand_pos], axis=1).astype(np.float32)
    right_joint_vel = np.concatenate([right_arm_vel, right_hand_vel], axis=1).astype(np.float32)

    obs = {
        "right_joint_pos": right_joint_pos,
        "right_joint_vel": right_joint_vel,
        "left_joint_pos": left_arm_pos.astype(np.float32),
        "left_joint_vel": left_arm_vel.astype(np.float32),
        "tip_force_norm": tip_force_norm.astype(np.float32),
        "prev_actions": prev_actions.astype(np.float32),
        # Backward-compatible raw terms kept as auxiliary observations.
        "right_arm_joint_pos": right_arm_pos.astype(np.float32),
        "left_arm_joint_pos": left_arm_pos.astype(np.float32),
        "right_hand_joint_pos": right_hand_pos.astype(np.float32),
        "right_hand_curl": right_curl.astype(np.float32),
        "tesollo_sensor": sensor.astype(np.float32),
        "datagen_info": _build_datagen_info(
            right_js=right_arm_pos,
            left_js=left_arm_pos,
            right_hand_curl=right_curl,
            topic_samples=topic_samples,
            query_ts=grid,
            object_pose_cfg=object_pose_cfg,
            subtask_signal_cfg=subtask_signal_cfg,
        ),
    }
    actor_obs = np.concatenate(
        [
            obs["right_joint_pos"],
            obs["right_joint_vel"],
            obs["left_joint_pos"],
            obs["left_joint_vel"],
            obs["tip_force_norm"],
            obs["prev_actions"],
        ],
        axis=1,
    ).astype(np.float32)
    if actor_obs.shape[1] != ACTOR_OBSERVATION_DIM:
        raise ValueError(
            f"actor_obs dim mismatch: expected={ACTOR_OBSERVATION_DIM}, got={actor_obs.shape[1]}"
        )
    obs["actor_obs"] = actor_obs
    return {
        "actions": actions,
        "obs": obs,
        "timestamps_ns": grid,
    }


def validate_demo_payload(demo: dict) -> None:
    actions = demo["actions"]
    validate_action_tensor(actions)

    obs = demo["obs"]
    required_obs_keys = set(OBSERVATION_DIM_BY_KEY.keys()) | {"actor_obs"}
    missing = required_obs_keys - set(obs.keys())
    if missing:
        raise ValueError(f"missing observation keys: {sorted(missing)}")

    num_steps = actions.shape[0]
    def _validate_obs_node(node: dict | np.ndarray, key_prefix: str) -> None:
        if isinstance(node, dict):
            for child_key, child in node.items():
                _validate_obs_node(child, f"{key_prefix}/{child_key}" if key_prefix else child_key)
            return

        if node.shape[0] != num_steps:
            raise ValueError(f"length mismatch for obs[{key_prefix}] vs actions")
        if not np.isfinite(node).all():
            raise ValueError(f"obs[{key_prefix}] contains NaN or inf")

    for key, value in obs.items():
        _validate_obs_node(value, key)
    if obs["actor_obs"].shape[-1] != ACTOR_OBSERVATION_DIM:
        raise ValueError("actor_obs dimension mismatch")


def write_hdf5(
    output_file: Path,
    demos: list[dict],
    quality_reports: list[dict],
    source_bag_dirs: list[str],
    object_pose_metadata: dict | None = None,
    subtask_signal_metadata: dict | None = None,
) -> None:
    def _write_obs_node(group: h5py.Group, key: str, value: dict | np.ndarray) -> None:
        if isinstance(value, dict):
            child = group.create_group(key)
            for child_key, child_value in value.items():
                _write_obs_node(child, child_key, child_value)
            return
        group.create_dataset(key, data=value, compression="gzip")

    output_file.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_file, "w") as h5:
        data_group = h5.create_group("data")
        data_group.attrs["env_args"] = json.dumps({"env_name": TASK_ID_MIMIC, "type": 2})
        data_group.attrs["source_type"] = "rosbag2_db3"
        data_group.attrs["source_bag_dirs"] = json.dumps(source_bag_dirs)
        data_group.attrs["quality_reports"] = json.dumps(quality_reports)
        if object_pose_metadata is not None:
            data_group.attrs["object_pose_config"] = json.dumps(object_pose_metadata)
        if subtask_signal_metadata is not None:
            data_group.attrs["subtask_signal_config"] = json.dumps(subtask_signal_metadata)

        total_samples = 0
        for demo_idx, demo in enumerate(demos):
            grp = data_group.create_group(f"demo_{demo_idx}")
            actions = demo["actions"]
            obs = demo["obs"]
            grp.attrs["num_samples"] = int(actions.shape[0])
            grp.attrs["success"] = True
            grp.create_dataset("actions", data=actions, compression="gzip")
            grp.create_dataset("timestamps_ns", data=demo["timestamps_ns"], compression="gzip")

            obs_group = grp.create_group("obs")
            for key, value in obs.items():
                _write_obs_node(obs_group, key, value)
            total_samples += int(actions.shape[0])

        data_group.attrs["total"] = total_samples


def convert_bags(
    bag_dirs: list[Path],
    output_file: Path,
    target_hz: int,
    max_drop_rate: float,
    max_gap_sec: float,
    skip_invalid_demos: bool,
    object_pose_cfg: ObjectPoseConfig,
    subtask_signal_cfg: SubtaskSignalConfig,
) -> tuple[int, int]:
    demos: list[dict] = []
    quality_reports: list[dict] = []
    source_bag_dirs: list[str] = []
    skipped = 0

    for bag_dir in bag_dirs:
        topic_samples = load_samples_from_bag_dir(bag_dir)
        quality = run_quality_gate(topic_samples, max_drop_rate=max_drop_rate, max_gap_sec=max_gap_sec)
        demo = build_demo(
            topic_samples,
            target_hz=target_hz,
            object_pose_cfg=object_pose_cfg,
            subtask_signal_cfg=subtask_signal_cfg,
        )
        try:
            validate_demo_payload(demo)
        except Exception:
            if skip_invalid_demos:
                skipped += 1
                continue
            raise
        demos.append(demo)
        quality_reports.append(quality)
        source_bag_dirs.append(str(bag_dir))

    if not demos:
        raise RuntimeError("no valid demos to write")
    write_hdf5(
        output_file=output_file,
        demos=demos,
        quality_reports=quality_reports,
        source_bag_dirs=source_bag_dirs,
        object_pose_metadata={
            "mode": object_pose_cfg.mode,
            "tf_topic": object_pose_cfg.tf_topic,
            "source_cup_frame": object_pose_cfg.source_cup_frame,
            "target_cup_frame": object_pose_cfg.target_cup_frame,
            "tf_reference_frame": object_pose_cfg.tf_reference_frame,
            "source_cup_pose_w": object_pose_cfg.source_cup_pose_w.tolist(),
            "target_cup_pose_w": object_pose_cfg.target_cup_pose_w.tolist(),
        },
        subtask_signal_metadata={
            "grasp_curl_threshold": subtask_signal_cfg.grasp_curl_threshold,
            "lift_z_margin": subtask_signal_cfg.lift_z_margin,
            "align_xy_threshold": subtask_signal_cfg.align_xy_threshold,
        },
    )
    return len(demos), skipped


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag-dir",
        dest="bag_dirs",
        action="append",
        required=True,
        help="Path to one rosbag directory (repeat for multiple demos).",
    )
    parser.add_argument("--output-file", required=True, help="Output HDF5 file path.")
    parser.add_argument("--target-hz", type=int, default=100, help="Resample frequency in Hz.")
    parser.add_argument(
        "--max-drop-rate",
        type=float,
        default=0.20,
        help="Per-topic allowed drop rate against expected message count.",
    )
    parser.add_argument(
        "--max-gap-sec",
        type=float,
        default=0.10,
        help="Maximum allowed timestamp gap on right arm stream.",
    )
    parser.add_argument(
        "--skip-invalid-demos",
        action="store_true",
        help="Skip demos that fail shape/NaN contract instead of fail-fast.",
    )
    parser.add_argument(
        "--object-pose-mode",
        choices=("static", "tf"),
        default="static",
        help="Object pose source for datagen_info/object_pose. static=hardcoded 4x4, tf=/tf,/tf_static chain with static fallback.",
    )
    parser.add_argument(
        "--source-cup-pose-w",
        default="1,0,0,0.35,0,1,0,-0.10,0,0,1,0.05,0,0,0,1",
        help="Row-major 4x4 world pose for source cup (static mode).",
    )
    parser.add_argument(
        "--target-cup-pose-w",
        default="1,0,0,0.35,0,1,0,0.10,0,0,1,0.05,0,0,0,1",
        help="Row-major 4x4 world pose for target cup (static mode).",
    )
    parser.add_argument("--tf-topic", default="/tf", help="TF topic name for tf object pose mode.")
    parser.add_argument("--source-cup-frame", default="source_cup", help="TF frame id for source cup.")
    parser.add_argument("--target-cup-frame", default="target_cup", help="TF frame id for target cup.")
    parser.add_argument("--tf-reference-frame", default="world", help="TF reference/world frame.")
    parser.add_argument(
        "--grasp-curl-threshold",
        type=float,
        default=0.55,
        help="Heuristic threshold for grasp_done from right-hand curl mean.",
    )
    parser.add_argument(
        "--lift-z-margin",
        type=float,
        default=0.06,
        help="Heuristic lift threshold: target_cup_z + lift_z_margin.",
    )
    parser.add_argument(
        "--align-xy-threshold",
        type=float,
        default=0.08,
        help="Heuristic align threshold (m) on ||eef_xy - target_cup_xy||.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_dirs = [Path(p).expanduser().resolve() for p in args.bag_dirs]
    output_file = Path(args.output_file).expanduser().resolve()
    object_pose_cfg = ObjectPoseConfig(
        mode=args.object_pose_mode,
        source_cup_pose_w=_parse_pose_matrix_arg(args.source_cup_pose_w, "--source-cup-pose-w"),
        target_cup_pose_w=_parse_pose_matrix_arg(args.target_cup_pose_w, "--target-cup-pose-w"),
        tf_topic=args.tf_topic,
        source_cup_frame=args.source_cup_frame,
        target_cup_frame=args.target_cup_frame,
        tf_reference_frame=args.tf_reference_frame,
    )
    subtask_signal_cfg = SubtaskSignalConfig(
        grasp_curl_threshold=float(args.grasp_curl_threshold),
        lift_z_margin=float(args.lift_z_margin),
        align_xy_threshold=float(args.align_xy_threshold),
    )
    converted, skipped = convert_bags(
        bag_dirs=bag_dirs,
        output_file=output_file,
        target_hz=args.target_hz,
        max_drop_rate=args.max_drop_rate,
        max_gap_sec=args.max_gap_sec,
        skip_invalid_demos=args.skip_invalid_demos,
        object_pose_cfg=object_pose_cfg,
        subtask_signal_cfg=subtask_signal_cfg,
    )
    print(
        f"[OK] converted={converted} skipped={skipped} output={output_file} "
        f"action_dim={ACTION_DIM} curl_slice={RIGHT_HAND_CURL_SLICE.start}:{RIGHT_HAND_CURL_SLICE.stop} "
        f"object_pose_mode={object_pose_cfg.mode}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
