#!/usr/bin/env python3
"""OpenArm right gripper -> Tesollo DG5F right hand teleop bridge.

Functions:
1) Convert /openarm/right/leader/gripper_state (0~0.04 default) to a
   synergy alpha in [0, 1], then interpolate between pose1 and pose2.
2) Publish command to /dg5f_right/rj_dg_pospid/reference as MultiDOFCommand.
3) Relay/publish Tesollo states for logging:
   - /tesollo/right/joint_states
   - /tesollo/right/position
   - /tesollo/right/velocity
   - /tesollo/right/torque
   - /tesollo/right/sensor  (fx,fy,fz,tx,ty,tz for fingertip 1..5)
"""
from __future__ import annotations

import sys
import time
from typing import Dict, List, Sequence

import rclpy
from control_msgs.msg import MultiDOFCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from tesollo_bridge_logic import (
    HAND_APPROACH_POSE,
    HAND_GRASP_POSE,
    JOINT_NAMES,
    compute_alpha,
    compute_target_positions,
)


def _as_float_list(values: Sequence[float]) -> List[float]:
    return [float(v) for v in values]


def _pad_or_trim(values: Sequence[float], target_len: int, fill: float) -> List[float]:
    vals = list(values)
    if len(vals) >= target_len:
        return vals[:target_len]
    return vals + [fill] * (target_len - len(vals))


class TesolloRightHandTeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__("tesollo_right_hand_teleop_bridge")

        default_pose1_rad = list(HAND_APPROACH_POSE)
        default_pose2_rad = list(HAND_GRASP_POSE)

        self.declare_parameter("input_topic", "/openarm/right/leader/gripper_state")
        self.declare_parameter("command_output_topic", "/dg5f_right/rj_dg_pospid/reference")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("leader_open_position", 0.010490577553978753)
        self.declare_parameter("leader_grasp_position", -1.1041809719996944)
        self.declare_parameter("invert_input", True)
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("hold_last_on_timeout", True)
        self.declare_parameter("pose1_rad", default_pose1_rad)
        self.declare_parameter("pose2_rad", default_pose2_rad)

        self.declare_parameter("tesollo_joint_state_topic", "/dg5f_right/joint_states")
        self.declare_parameter("tesollo_sensor_prefix", "/dg5f_right/fingertip_")
        self.declare_parameter("relay_joint_state_topic", "/tesollo/right/joint_states")
        self.declare_parameter("relay_position_topic", "/tesollo/right/position")
        self.declare_parameter("relay_velocity_topic", "/tesollo/right/velocity")
        self.declare_parameter("relay_torque_topic", "/tesollo/right/torque")
        self.declare_parameter("relay_sensor_topic", "/tesollo/right/sensor")
        self.declare_parameter("auto_calibrate_ft_sensor", True)
        self.declare_parameter(
            "ft_offset_service",
            "/dg5f_right/delto_hardware_interface_node/set_ft_sensor_offset",
        )
        self.declare_parameter("ft_offset_wait_timeout_sec", 15.0)

        input_topic: str = self.get_parameter("input_topic").value
        command_output_topic: str = self.get_parameter("command_output_topic").value
        publish_rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._open_pos: float = self.get_parameter("leader_open_position").value
        self._grasp_pos: float = self.get_parameter("leader_grasp_position").value
        self._invert: bool = self.get_parameter("invert_input").value
        self._timeout: float = self.get_parameter("input_timeout_sec").value
        self._hold_last: bool = self.get_parameter("hold_last_on_timeout").value

        pose1_rad = _as_float_list(self.get_parameter("pose1_rad").value)
        pose2_rad = _as_float_list(self.get_parameter("pose2_rad").value)
        if len(pose1_rad) != len(JOINT_NAMES) or len(pose2_rad) != len(JOINT_NAMES):
            self.get_logger().warn(
                "pose1_rad/pose2_rad length must be 20. Falling back to defaults."
            )
            pose1_rad = default_pose1_rad
            pose2_rad = default_pose2_rad

        self._pose1_rad = pose1_rad
        self._pose2_rad = pose2_rad

        self._disabled = abs(self._grasp_pos - self._open_pos) < 1e-9
        if self._disabled:
            self.get_logger().error(
                "leader_open_position == leader_grasp_position: command publishing disabled."
            )

        self._last_target: List[float] = list(self._pose1_rad)
        self._last_input_time = time.monotonic()
        self._has_received = False

        self._cmd_pub = self.create_publisher(MultiDOFCommand, command_output_topic, 10)
        self.create_subscription(JointState, input_topic, self._leader_gripper_callback, 10)

        tesollo_joint_state_topic = self.get_parameter("tesollo_joint_state_topic").value
        self._js_relay_pub = self.create_publisher(
            JointState, self.get_parameter("relay_joint_state_topic").value, 10
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("relay_position_topic").value, 10
        )
        self._vel_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("relay_velocity_topic").value, 10
        )
        self._torque_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("relay_torque_topic").value, 10
        )
        self._sensor_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("relay_sensor_topic").value, 10
        )
        self.create_subscription(
            JointState, tesollo_joint_state_topic, self._tesollo_joint_state_callback, 10
        )

        self._fingertip_wrench: Dict[int, WrenchStamped] = {}
        sensor_prefix = self.get_parameter("tesollo_sensor_prefix").value
        for i in range(1, 6):
            topic = f"{sensor_prefix}{i}_broadcaster/wrench"
            self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, idx=i: self._fingertip_wrench_callback(idx, msg),
                10,
            )

        self.create_timer(1.0 / publish_rate_hz, self._command_timer_callback)

        self._ft_offset_timer = None
        self._ft_offset_done = False
        self._ft_offset_inflight = False
        self._ft_offset_deadline = time.monotonic()
        self._ft_offset_last_wait_log = 0.0
        ft_offset_service: str = self.get_parameter("ft_offset_service").value
        self._ft_offset_client = self.create_client(Trigger, ft_offset_service)
        self._auto_ft_offset: bool = self.get_parameter("auto_calibrate_ft_sensor").value
        ft_offset_wait_timeout_sec: float = self.get_parameter("ft_offset_wait_timeout_sec").value
        self._ft_offset_deadline = time.monotonic() + max(0.0, ft_offset_wait_timeout_sec)
        if self._auto_ft_offset:
            self._ft_offset_timer = self.create_timer(0.5, self._startup_ft_offset_callback)

        self.get_logger().info(
            "Tesollo teleop bridge started. "
            f"input={input_topic}, cmd_out={command_output_topic}, "
            f"joint_in={tesollo_joint_state_topic}"
        )
        if self._auto_ft_offset:
            self.get_logger().info(
                f"Auto F/T zero enabled. Waiting for service: {ft_offset_service}"
            )

    def _leader_gripper_callback(self, msg: JointState) -> None:
        if self._disabled or not msg.position:
            return

        alpha = compute_alpha(
            msg.position[0],
            self._open_pos,
            self._grasp_pos,
            self._invert,
        )
        if alpha is None:
            return

        self._last_target = compute_target_positions(alpha, self._pose1_rad, self._pose2_rad)
        self._last_input_time = time.monotonic()
        self._has_received = True

    def _command_timer_callback(self) -> None:
        if self._disabled or not self._has_received:
            return
        if (time.monotonic() - self._last_input_time) > self._timeout and not self._hold_last:
            return

        out = MultiDOFCommand()
        out.dof_names = JOINT_NAMES
        out.values = self._last_target
        out.values_dot = [0.0] * len(JOINT_NAMES)
        self._cmd_pub.publish(out)

    def _startup_ft_offset_callback(self) -> None:
        if self._ft_offset_done:
            if self._ft_offset_timer is not None:
                self._ft_offset_timer.cancel()
            return
        if self._ft_offset_inflight:
            return

        now = time.monotonic()
        if now > self._ft_offset_deadline:
            self.get_logger().warn(
                "Auto F/T zero skipped: offset service was not available before timeout."
            )
            self._ft_offset_done = True
            if self._ft_offset_timer is not None:
                self._ft_offset_timer.cancel()
            return

        if not self._ft_offset_client.service_is_ready():
            if (now - self._ft_offset_last_wait_log) >= 2.0:
                self._ft_offset_last_wait_log = now
                self.get_logger().info("Waiting for F/T offset service...")
            return

        self._ft_offset_inflight = True
        future = self._ft_offset_client.call_async(Trigger.Request())
        future.add_done_callback(self._ft_offset_response_callback)

    def _ft_offset_response_callback(self, future) -> None:
        self._ft_offset_inflight = False
        self._ft_offset_done = True
        if self._ft_offset_timer is not None:
            self._ft_offset_timer.cancel()

        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport error
            self.get_logger().warn(f"Auto F/T zero failed: {exc}")
            return

        if response is None:
            self.get_logger().warn("Auto F/T zero failed: empty response.")
            return
        if response.success:
            self.get_logger().info(f"Auto F/T zero done: {response.message}")
            return
        self.get_logger().warn(f"Auto F/T zero rejected: {response.message}")

    def _tesollo_joint_state_callback(self, msg: JointState) -> None:
        self._js_relay_pub.publish(msg)

        target_len = len(msg.name)
        pos = _pad_or_trim(msg.position, target_len, float("nan"))
        vel = _pad_or_trim(msg.velocity, target_len, float("nan"))
        eff = _pad_or_trim(msg.effort, target_len, float("nan"))

        pos_msg = Float64MultiArray()
        pos_msg.data = pos
        self._pos_pub.publish(pos_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = vel
        self._vel_pub.publish(vel_msg)

        torque_msg = Float64MultiArray()
        torque_msg.data = eff
        self._torque_pub.publish(torque_msg)

    def _fingertip_wrench_callback(self, finger_idx: int, msg: WrenchStamped) -> None:
        self._fingertip_wrench[finger_idx] = msg
        self._publish_sensor_array()

    def _publish_sensor_array(self) -> None:
        arr: List[float] = []
        nan = float("nan")
        for i in range(1, 6):
            wrench_msg = self._fingertip_wrench.get(i)
            if wrench_msg is None:
                arr.extend([nan, nan, nan, nan, nan, nan])
                continue
            w = wrench_msg.wrench
            arr.extend([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

        msg = Float64MultiArray()
        msg.data = arr
        self._sensor_pub.publish(msg)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TesolloRightHandTeleopBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
