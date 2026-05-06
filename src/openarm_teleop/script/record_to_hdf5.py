#!/usr/bin/env python3
"""record_to_hdf5.py — Real-robot teleoperation → IsaacLab Mimic HDF5 recorder.

Records ROS2 topics from OpenArm bilateral teleoperation to HDF5 format
compatible with IsaacLab Mimic's data generation pipeline.

Topics subscribed (ROS_DOMAIN_ID=126):
    /openarm/left/joint_states          sensor_msgs/JointState
    /openarm/right/joint_states         sensor_msgs/JointState
    /openarm/left/leader/gripper_state  sensor_msgs/JointState
    /dg5f_right/rj_dg_pospid/reference  control_msgs/MultiDOFCommand
    /dg5f_right/joint_states            sensor_msgs/JointState
    /tesollo/right/sensor               std_msgs/Float64MultiArray

HDF5 output (robomimic / IsaacLab Mimic compatible):
    data/
      demo_N/
        actions          float32 [T, 34]  left(7)+right(7)+dg5f_ref(20)
        obs/
          left_joint_pos      float32 [T, 7]
          right_joint_pos     float32 [T, 7]
          left_gripper        float32 [T, 1]
          tesollo_joint_states float32 [T, 20]
          tesollo_sensor      float32 [T, 30]  fx,fy,fz,tx,ty,tz x5 fingers
        leader_obs/
          left_joint_pos  float32 [T, 7]
          right_joint_pos float32 [T, 7]
          left_gripper    float32 [T, 1]
        attrs: num_samples, success

Usage:
    ROS_DOMAIN_ID=126 python3 script/record_to_hdf5.py --output datasets/demo.hdf5
    ROS_DOMAIN_ID=126 python3 script/record_to_hdf5.py --output datasets/demo.hdf5 --hz 10

Keyboard controls:
    Space   start / stop episode
    q / Q   quit and save
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from typing import Optional

import h5py
import numpy as np

# ── dimension constants ────────────────────────────────────────────────────────
ARM_DOF = 7        # per arm (OpenArm 7-DOF)
GRIPPER_DOF = 1    # leader gripper scalar
TESOLLO_DOF = 20   # DG5F hand joints
SENSOR_DIM = 30    # 6 wrench values × 5 fingertips
ACTION_DIM = ARM_DOF + ARM_DOF + TESOLLO_DOF  # 34


# ══════════════════════════════════════════════════════════════════════════════
# TopicBuffer
# ══════════════════════════════════════════════════════════════════════════════


class TopicBuffer:
    """Thread-safe store for the latest numpy array received from a ROS2 topic."""

    def __init__(self) -> None:
        self._data: Optional[np.ndarray] = None
        self._lock = threading.Lock()

    def update(self, value: np.ndarray) -> None:
        with self._lock:
            self._data = value

    def get(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._data


# ══════════════════════════════════════════════════════════════════════════════
# HDF5Writer
# ══════════════════════════════════════════════════════════════════════════════


class HDF5Writer:
    """Writes episodes to an IsaacLab Mimic / robomimic compatible HDF5 file."""

    def __init__(self, file_path: str, env_name: str = "OpenArmBilateral") -> None:
        dir_path = os.path.dirname(os.path.abspath(file_path))
        os.makedirs(dir_path, exist_ok=True)

        self._f = h5py.File(file_path, "w")
        self._data = self._f.create_group("data")
        self._data.attrs["total"] = 0
        self._data.attrs["env_args"] = json.dumps({"env_name": env_name, "type": 2})
        self._demo_count = 0

    def write_episode(self, episode_data: dict, success: bool) -> None:
        if not episode_data:
            return

        actions = episode_data["actions"]
        n_steps = len(actions)

        grp = self._data.create_group(f"demo_{self._demo_count}")
        grp.attrs["num_samples"] = n_steps
        grp.attrs["success"] = success

        grp.create_dataset("actions", data=actions, compression="gzip")

        obs_grp = grp.create_group("obs")
        for key, val in episode_data.get("obs", {}).items():
            obs_grp.create_dataset(key, data=val, compression="gzip")

        leader_grp = grp.create_group("leader_obs")
        for key, val in episode_data.get("leader_obs", {}).items():
            leader_grp.create_dataset(key, data=val, compression="gzip")

        self._data.attrs["total"] = int(self._data.attrs["total"]) + n_steps
        self._demo_count += 1

    def flush(self) -> None:
        self._f.flush()

    def close(self) -> None:
        if self._f.id.valid:
            self._f.close()

    def __del__(self) -> None:
        self.close()


# ══════════════════════════════════════════════════════════════════════════════
# EpisodeRecorder
# ══════════════════════════════════════════════════════════════════════════════

_ZEROS: dict[str, np.ndarray] = {
    "left_joint_states": np.zeros(ARM_DOF, dtype=np.float32),
    "right_joint_states": np.zeros(ARM_DOF, dtype=np.float32),
    "left_gripper": np.zeros(GRIPPER_DOF, dtype=np.float32),
    "dg5f_reference": np.zeros(TESOLLO_DOF, dtype=np.float32),
    "dg5f_joint_states": np.zeros(TESOLLO_DOF, dtype=np.float32),
    "tesollo_sensor": np.zeros(SENSOR_DIM, dtype=np.float32),
}


class EpisodeRecorder:
    """Accumulates sampled data into episode dicts for HDF5 export."""

    def __init__(self, buffers: dict[str, TopicBuffer]) -> None:
        self._buffers = buffers
        self._recording = False
        self._accum: dict[str, list[np.ndarray]] = {}

    @property
    def is_recording(self) -> bool:
        return self._recording

    def start(self) -> None:
        self._recording = True
        self._accum = {
            "actions": [],
            "obs_left": [], "obs_right": [], "obs_left_gripper": [],
            "obs_tesollo_js": [], "obs_tesollo_sensor": [],
            "leader_left": [], "leader_right": [], "leader_gripper": [],
        }

    def stop(self) -> dict:
        if not self._recording:
            raise RuntimeError("stop() called while not recording")
        self._recording = False
        return self._build_episode()

    def sample(self) -> None:
        if not self._recording:
            return

        def _read(key: str) -> np.ndarray:
            val = self._buffers[key].get()
            return val if val is not None else _ZEROS[key].copy()

        left = _read("left_joint_states").astype(np.float32)
        right = _read("right_joint_states").astype(np.float32)
        gripper = _read("left_gripper").astype(np.float32)
        dg5f_ref = _read("dg5f_reference").astype(np.float32)
        dg5f_js = _read("dg5f_joint_states").astype(np.float32)
        sensor = _read("tesollo_sensor").astype(np.float32)

        self._accum["actions"].append(np.concatenate([left, right, dg5f_ref]))
        self._accum["obs_left"].append(left)
        self._accum["obs_right"].append(right)
        self._accum["obs_left_gripper"].append(gripper)
        self._accum["obs_tesollo_js"].append(dg5f_js)
        self._accum["obs_tesollo_sensor"].append(sensor)
        self._accum["leader_left"].append(left.copy())
        self._accum["leader_right"].append(right.copy())
        self._accum["leader_gripper"].append(gripper.copy())

    def _build_episode(self) -> dict:
        _empty_shapes = {
            "actions": (0, ACTION_DIM),
            "obs_left": (0, ARM_DOF), "obs_right": (0, ARM_DOF),
            "obs_left_gripper": (0, GRIPPER_DOF),
            "obs_tesollo_js": (0, TESOLLO_DOF), "obs_tesollo_sensor": (0, SENSOR_DIM),
            "leader_left": (0, ARM_DOF), "leader_right": (0, ARM_DOF),
            "leader_gripper": (0, GRIPPER_DOF),
        }

        def stack(key: str) -> np.ndarray:
            lst = self._accum[key]
            if not lst:
                return np.empty(_empty_shapes[key], dtype=np.float32)
            return np.stack(lst, axis=0)

        return {
            "actions": stack("actions"),
            "obs": {
                "left_joint_pos": stack("obs_left"),
                "right_joint_pos": stack("obs_right"),
                "left_gripper": stack("obs_left_gripper"),
                "tesollo_joint_states": stack("obs_tesollo_js"),
                "tesollo_sensor": stack("obs_tesollo_sensor"),
            },
            "leader_obs": {
                "left_joint_pos": stack("leader_left"),
                "right_joint_pos": stack("leader_right"),
                "left_gripper": stack("leader_gripper"),
            },
        }


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 subscriber node (runs in background thread)
# ══════════════════════════════════════════════════════════════════════════════


def _run_ros_node(buffers: dict[str, TopicBuffer], domain_id: int) -> None:
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)

    import rclpy
    from control_msgs.msg import MultiDOFCommand
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    class _RecorderNode(Node):
        def __init__(self) -> None:
            super().__init__("hdf5_recorder")
            qos = rclpy.qos.QoSProfile(depth=1)

            def js_cb(buf: TopicBuffer, dof: int):
                def _cb(msg: JointState) -> None:
                    pos = list(msg.position)
                    n = len(pos)
                    arr = np.array(
                        pos[:dof] if n >= dof else pos + [0.0] * (dof - n),
                        dtype=np.float32,
                    )
                    buf.update(arr)
                return _cb

            self.create_subscription(
                JointState, "/openarm/left/joint_states",
                js_cb(buffers["left_joint_states"], ARM_DOF), qos)
            self.create_subscription(
                JointState, "/openarm/right/joint_states",
                js_cb(buffers["right_joint_states"], ARM_DOF), qos)
            self.create_subscription(
                JointState, "/dg5f_right/joint_states",
                js_cb(buffers["dg5f_joint_states"], TESOLLO_DOF), qos)

            def gripper_cb(msg: JointState) -> None:
                val = msg.position[0] if msg.position else 0.0
                buffers["left_gripper"].update(np.array([val], dtype=np.float32))

            self.create_subscription(
                JointState, "/openarm/left/leader/gripper_state", gripper_cb, qos)

            def dg5f_ref_cb(msg: MultiDOFCommand) -> None:
                pos = list(msg.values) if hasattr(msg, "values") else []
                n = len(pos)
                arr = np.array(
                    pos[:TESOLLO_DOF] if n >= TESOLLO_DOF else pos + [0.0] * (TESOLLO_DOF - n),
                    dtype=np.float32,
                )
                buffers["dg5f_reference"].update(arr)

            self.create_subscription(
                MultiDOFCommand, "/dg5f_right/rj_dg_pospid/reference", dg5f_ref_cb, qos)

            def sensor_cb(msg: Float64MultiArray) -> None:
                data = list(msg.data)
                n = len(data)
                arr = np.array(
                    data[:SENSOR_DIM] if n >= SENSOR_DIM else data + [0.0] * (SENSOR_DIM - n),
                    dtype=np.float32,
                )
                buffers["tesollo_sensor"].update(arr)

            self.create_subscription(
                Float64MultiArray, "/tesollo/right/sensor", sensor_cb, qos)

    rclpy.init()
    node = _RecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════════════════
# Keyboard (non-blocking, raw terminal)
# ══════════════════════════════════════════════════════════════════════════════


def _keyboard_loop(on_space, on_quit) -> None:
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == " ":
                on_space()
            elif ch in ("q", "Q", "\x03"):
                on_quit()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════


def main() -> None:
    parser = argparse.ArgumentParser(description="Record OpenArm teleoperation to HDF5.")
    parser.add_argument("--output", default="./datasets/demo.hdf5",
                        help="HDF5 output file path.")
    parser.add_argument("--hz", type=float, default=10.0,
                        help="Sampling rate in Hz (default: 10).")
    parser.add_argument("--domain", type=int, default=126,
                        help="ROS_DOMAIN_ID (default: 126).")
    parser.add_argument("--env-name", default="OpenArmBilateral",
                        help="Environment name stored in HDF5 metadata.")
    args = parser.parse_args()

    buffers: dict[str, TopicBuffer] = {
        "left_joint_states": TopicBuffer(),
        "right_joint_states": TopicBuffer(),
        "left_gripper": TopicBuffer(),
        "dg5f_reference": TopicBuffer(),
        "dg5f_joint_states": TopicBuffer(),
        "tesollo_sensor": TopicBuffer(),
    }

    writer = HDF5Writer(args.output, env_name=args.env_name)
    recorder = EpisodeRecorder(buffers=buffers)
    period = 1.0 / args.hz
    quit_flag = threading.Event()
    episode_count = 0

    def toggle_recording():
        nonlocal episode_count
        if recorder.is_recording:
            episode = recorder.stop()
            n = len(episode["actions"])
            if n > 0:
                writer.write_episode(episode, success=True)
                writer.flush()
                episode_count += 1
                print(f"\r[demo_{episode_count - 1}] {n} steps saved. "
                      f"Total: {episode_count}          ")
            else:
                print("\rEmpty episode discarded.                    ")
        else:
            recorder.start()
            print("\r[RECORDING] Press Space to stop...            ", end="", flush=True)

    def quit_app():
        nonlocal episode_count
        print("\nSaving and exiting...")
        if recorder.is_recording:
            episode = recorder.stop()
            if len(episode["actions"]) > 0:
                writer.write_episode(episode, success=True)
                episode_count += 1
        writer.close()
        quit_flag.set()

    ros_thread = threading.Thread(
        target=_run_ros_node, args=(buffers, args.domain), daemon=True)
    ros_thread.start()

    kb_thread = threading.Thread(
        target=_keyboard_loop, args=(toggle_recording, quit_app), daemon=True)
    kb_thread.start()

    print(f"Recording to : {args.output}")
    print(f"Sample rate  : {args.hz} Hz  |  ROS_DOMAIN_ID={args.domain}")
    print("Press [Space] to start/stop episode,  [q] to quit\n")

    try:
        while not quit_flag.is_set():
            t0 = time.monotonic()
            recorder.sample()
            sleep_time = period - (time.monotonic() - t0)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        quit_app()

    print(f"\nDone. {episode_count} episode(s) saved to {args.output}")


if __name__ == "__main__":
    main()
