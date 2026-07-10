#!/usr/bin/env python3
"""Real2Sim identification bag(db3) → HDF5.

`db3_to_pour_mimic_hdf5.py`는 pour 전용 토픽/데이터셋에 묶여 있어 재사용하지 않는다.
여기서는 명령/측정 두 토픽만 읽는다.

기본 대상 (손, 100 Hz):
    command  /dg5f_right/rj_dg_pospid/reference   control_msgs/MultiDOFCommand
    measured /dg5f_right/joint_states             sensor_msgs/JointState

isaacsim_bridge를 거치는 `/isaacsim/right_hand_cmd`를 command로 쓰면 안 된다.
브리지가 위치 목표를 trajectory_time_sec 짜리 궤적으로 보간하므로, 식별 대상이
액추에이터가 아니라 브리지의 보간 필터가 된다.

사용:
  python3 db3_to_identification_hdf5.py \
      --bag-dir bags/real2sim_identification/run_001 \
      --output datasets/real2sim_identification_100hz.hdf5
"""

from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from identification_hdf5 import (  # noqa: E402
    IdentificationError,
    NamedSample,
    assert_identifiable,
    build_identification_track,
    write_identification_hdf5,
)

DEFAULT_COMMAND_TOPIC = "/dg5f_right/rj_dg_pospid/reference"
DEFAULT_MEASURED_TOPIC = "/dg5f_right/joint_states"


def _topic_types(connection: sqlite3.Connection) -> dict[str, str]:
    rows = connection.execute("SELECT name, type FROM topics").fetchall()
    return {name: type_name for name, type_name in rows}


def _import_message_class(type_name: str):
    package, kind, message = type_name.split("/")
    module = __import__(f"{package}.{kind}", fromlist=[message])
    return getattr(module, message)


def load_controller_joint_order(config_path: Path, controller_name: str) -> tuple[str, ...]:
    """ros2_control 컨트롤러 yaml에서 joints 순서를 읽는다.

    팔 명령(`Float64MultiArray`)에는 이름이 없다. 이 순서가 어긋나면 관절이 뒤섞인 채
    MSE가 계산되고, 그 결과는 낮아도 아무 의미가 없다.
    """
    import yaml

    document = yaml.safe_load(config_path.read_text())
    for key, body in document.items():
        if key.split("/")[-1] != controller_name:
            continue
        joints = body.get("ros__parameters", {}).get("joints")
        if joints:
            return tuple(joints)
    raise IdentificationError(f"controller '{controller_name}' has no 'joints' in {config_path}")


def _to_named_sample(
    timestamp_ns: int,
    message,
    joint_names: tuple[str, ...] | None = None,
) -> NamedSample:
    """MultiDOFCommand / JointState / Float64MultiArray 를 하나의 표현으로 모은다.

    앞의 둘은 이름을 들고 다니므로 순서를 추측할 필요가 없다.
    Float64MultiArray(팔 forward_position_controller)만 joint_names를 밖에서 받아야 한다.
    """
    if hasattr(message, "dof_names"):  # control_msgs/MultiDOFCommand
        return NamedSample(
            timestamp_ns=timestamp_ns,
            names=tuple(message.dof_names),
            positions=tuple(float(v) for v in message.values),
        )
    if hasattr(message, "name"):  # sensor_msgs/JointState
        velocities = tuple(float(v) for v in message.velocity) if len(message.velocity) else None
        return NamedSample(
            timestamp_ns=timestamp_ns,
            names=tuple(message.name),
            positions=tuple(float(v) for v in message.position),
            velocities=velocities,
        )
    if hasattr(message, "data"):  # std_msgs/Float64MultiArray
        if not joint_names:
            raise IdentificationError(
                "Float64MultiArray carries no joint names; "
                "--command-joint-order 또는 --controller-config 로 순서를 명시하라"
            )
        values = tuple(float(v) for v in message.data)
        if len(values) != len(joint_names):
            raise IdentificationError(
                f"Float64MultiArray has {len(values)} values but {len(joint_names)} joint names given"
            )
        return NamedSample(timestamp_ns=timestamp_ns, names=joint_names, positions=values)
    raise IdentificationError(f"unsupported message type: {type(message).__name__}")


def read_topic(
    bag_dir: Path,
    topic: str,
    joint_names: tuple[str, ...] | None = None,
) -> list[NamedSample]:
    from rclpy.serialization import deserialize_message

    db3_files = sorted(bag_dir.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"no .db3 file under: {bag_dir}")

    samples: list[NamedSample] = []
    for db3 in db3_files:
        connection = sqlite3.connect(str(db3))
        try:
            types = _topic_types(connection)
            if topic not in types:
                raise IdentificationError(
                    f"topic '{topic}' not in bag {db3.name}. 녹화된 토픽: {sorted(types)}"
                )
            message_class = _import_message_class(types[topic])
            rows = connection.execute(
                "SELECT m.timestamp, m.data FROM messages m "
                "JOIN topics t ON m.topic_id = t.id WHERE t.name = ? ORDER BY m.timestamp",
                (topic,),
            )
            for timestamp_ns, blob in rows:
                message = deserialize_message(blob, message_class)
                samples.append(_to_named_sample(int(timestamp_ns), message, joint_names))
        finally:
            connection.close()

    if not samples:
        raise IdentificationError(f"topic '{topic}' has no messages in {bag_dir}")
    return samples


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert an identification bag to HDF5.")
    parser.add_argument("--bag-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--measured-topic", default=DEFAULT_MEASURED_TOPIC)
    parser.add_argument("--dt", type=float, default=0.01, help="리샘플 주기 [s] (손 100 Hz)")
    parser.add_argument(
        "--command-joint-order",
        default=None,
        help="쉼표 구분. 팔의 Float64MultiArray처럼 이름이 없는 명령에만 필요하다",
    )
    parser.add_argument("--controller-config", type=Path, default=None, help="ros2_control 컨트롤러 yaml")
    parser.add_argument("--controller-name", default=None, help="예: right_forward_position_controller")
    parser.add_argument(
        "--allow-unexcited",
        action="store_true",
        help="명령이 거의 움직이지 않아도 통과시킨다 (진단용)",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    joint_order: tuple[str, ...] | None = None
    if args.command_joint_order:
        joint_order = tuple(n.strip() for n in args.command_joint_order.split(","))
    elif args.controller_config and args.controller_name:
        joint_order = load_controller_joint_order(args.controller_config, args.controller_name)
    if joint_order:
        print(f"[r2s] command joint order ({len(joint_order)}): {', '.join(joint_order)}")

    command = read_topic(args.bag_dir, args.command_topic, joint_order)
    measured = read_topic(args.bag_dir, args.measured_topic)
    print(f"[r2s] command  {args.command_topic}: {len(command)} msgs")
    print(f"[r2s] measured {args.measured_topic}: {len(measured)} msgs")

    track = build_identification_track(command, measured, dt=args.dt)
    print(f"[r2s] resampled to {track.q_cmd.shape[0]} steps × {len(track.joint_names)} joints @ {args.dt}s")

    try:
        assert_identifiable(track)
    except IdentificationError as error:
        if not args.allow_unexcited:
            print(f"[r2s] ERROR: {error}", file=sys.stderr)
            return 1
        print(f"[r2s] WARNING: {error}", file=sys.stderr)

    write_identification_hdf5(
        args.output,
        track,
        attrs={"source_bag": str(args.bag_dir), "command_topic": args.command_topic},
    )
    amplitude = np.ptp(track.q_cmd, axis=0)
    print(f"[r2s] command amplitude: min {amplitude.min():.4f} max {amplitude.max():.4f} rad")
    print(f"[r2s] wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
