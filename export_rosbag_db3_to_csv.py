#!/usr/bin/env python3
import argparse
import array
import base64
import csv
import json
import sqlite3
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def topic_to_filename(topic_name: str) -> str:
    return topic_name.strip("/").replace("/", "__") or "root"


def ros_value_to_builtin(value: Any) -> Any:
    if hasattr(value, "get_fields_and_field_types"):
        result = {}
        for field_name in value.get_fields_and_field_types().keys():
            result[field_name] = ros_value_to_builtin(getattr(value, field_name))
        return result
    if isinstance(value, (list, tuple)):
        return [ros_value_to_builtin(v) for v in value]
    if isinstance(value, array.array):
        return list(value)
    if isinstance(value, bytes):
        return base64.b64encode(value).decode("ascii")
    return value


def read_topics(conn: sqlite3.Connection) -> Dict[int, Tuple[str, str]]:
    cursor = conn.cursor()
    cursor.execute("SELECT id, name, type FROM topics")
    rows = cursor.fetchall()
    return {topic_id: (name, msg_type) for topic_id, name, msg_type in rows}


def export_bag_dir(bag_dir: Path, out_root: Path) -> None:
    db3_files = sorted(bag_dir.glob("*.db3"))
    if not db3_files:
        print(f"[WARN] no .db3 in: {bag_dir}")
        return

    bag_out_dir = out_root / bag_dir.name
    bag_out_dir.mkdir(parents=True, exist_ok=True)

    writers: Dict[str, csv.DictWriter] = {}
    handles = []
    msg_classes: Dict[str, Any] = {}

    try:
        for db3_file in db3_files:
            conn = sqlite3.connect(str(db3_file))
            topics_by_id = read_topics(conn)

            for _, (topic_name, msg_type) in topics_by_id.items():
                if topic_name not in msg_classes:
                    msg_classes[topic_name] = get_message(msg_type)
                if topic_name not in writers:
                    csv_path = bag_out_dir / f"{topic_to_filename(topic_name)}.csv"
                    f = open(csv_path, "w", newline="", encoding="utf-8")
                    handles.append(f)
                    writer = csv.DictWriter(
                        f, fieldnames=["timestamp_ns", "topic", "message_json"]
                    )
                    writer.writeheader()
                    writers[topic_name] = writer

            cursor = conn.cursor()
            cursor.execute(
                "SELECT timestamp, topic_id, data FROM messages ORDER BY timestamp ASC"
            )
            for timestamp_ns, topic_id, raw_data in cursor:
                topic_name, _ = topics_by_id[topic_id]
                msg_class = msg_classes[topic_name]
                writer = writers[topic_name]

                try:
                    msg_obj = deserialize_message(raw_data, msg_class)
                    payload = ros_value_to_builtin(msg_obj)
                    payload_json = json.dumps(payload, ensure_ascii=False)
                except Exception as exc:
                    payload_json = json.dumps(
                        {
                            "_deserialize_error": str(exc),
                            "_raw_data_b64": base64.b64encode(raw_data).decode("ascii"),
                        },
                        ensure_ascii=False,
                    )

                writer.writerow(
                    {
                        "timestamp_ns": timestamp_ns,
                        "topic": topic_name,
                        "message_json": payload_json,
                    }
                )

            conn.close()

        print(f"[OK] {bag_dir} -> {bag_out_dir}")
    finally:
        for f in handles:
            f.close()


def collect_bag_dirs(inputs: List[str]) -> List[Path]:
    dirs = []
    for item in inputs:
        p = Path(item).expanduser().resolve()
        if p.is_dir():
            dirs.append(p)
        else:
            print(f"[WARN] not a directory, skipped: {item}")
    return dirs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export ROS2 rosbag2 sqlite3 (.db3) files to topic CSV files."
    )
    parser.add_argument(
        "bag_dirs",
        nargs="+",
        help="Bag directories (each containing one or more .db3 files).",
    )
    parser.add_argument(
        "--out-root",
        default="csv_export",
        help="Output root directory (default: ./csv_export).",
    )
    args = parser.parse_args()

    bag_dirs = collect_bag_dirs(args.bag_dirs)
    if not bag_dirs:
        print("[ERROR] no valid bag directories provided.")
        return 1

    out_root = Path(args.out_root).expanduser().resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    for bag_dir in bag_dirs:
        export_bag_dir(bag_dir, out_root)

    print(f"[DONE] output root: {out_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
