#!/usr/bin/env python3
"""Tesollo DG5F right hand setting bridge node.

Subscribes to /openarm/right/leader/gripper_state and publishes
MultiDOFCommand to /dg5f_right/rj_dg_pospid/reference at 100 Hz.
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState

from tesollo_bridge_logic import (
    HAND_APPROACH_POSE,
    HAND_GRASP_POSE,
    JOINT_NAMES,
    compute_alpha,
    compute_target_positions,
)

Q_INITIAL: list[float] = list(HAND_APPROACH_POSE)
Q_GRASP: list[float] = list(HAND_GRASP_POSE)


class TesolloBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("tesollo_right_hand_setting_bridge")

        self.declare_parameter("input_topic", "/openarm/right/leader/gripper_state")
        self.declare_parameter("output_topic", "/dg5f_right/rj_dg_pospid/reference")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("leader_open_position", 0.010490577553978753)
        self.declare_parameter("leader_grasp_position", -1.1041809719996944)
        self.declare_parameter("invert_input", True)
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("hold_last_on_timeout", True)

        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value
        rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._open_pos: float = self.get_parameter("leader_open_position").value
        self._grasp_pos: float = self.get_parameter("leader_grasp_position").value
        self._invert: bool = self.get_parameter("invert_input").value
        self._timeout: float = self.get_parameter("input_timeout_sec").value
        self._hold_last: bool = self.get_parameter("hold_last_on_timeout").value

        self._disabled: bool = abs(self._grasp_pos - self._open_pos) < 1e-9
        if self._disabled:
            self.get_logger().error(
                "leader_open_position == leader_grasp_position: publishing disabled."
            )

        self._last_target: list[float] = list(Q_INITIAL)
        self._last_input_time: float = time.monotonic()
        self._has_received: bool = False

        self._pub = self.create_publisher(MultiDOFCommand, output_topic, 10)
        self._sub = self.create_subscription(
            JointState, input_topic, self._gripper_callback, 10
        )
        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._timer_callback)

    def _gripper_callback(self, msg: JointState) -> None:
        if self._disabled:
            return
        if not msg.position:
            return

        leader_pos = msg.position[0]
        alpha = compute_alpha(leader_pos, self._open_pos, self._grasp_pos, self._invert)
        if alpha is None:
            return

        self._last_target = compute_target_positions(alpha, Q_INITIAL, Q_GRASP)
        self._last_input_time = time.monotonic()
        self._has_received = True

    def _timer_callback(self) -> None:
        if self._disabled:
            return

        if not self._has_received:
            return

        elapsed = time.monotonic() - self._last_input_time
        if elapsed > self._timeout and not self._hold_last:
            return

        out = MultiDOFCommand()
        out.dof_names = JOINT_NAMES
        out.values = self._last_target
        out.values_dot = [0.0] * len(JOINT_NAMES)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TesolloBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
