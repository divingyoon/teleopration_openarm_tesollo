#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from dg_msgs.srv import SetGripperSystem
from dg_msgs.msg import GripperSystemSetting


def main(args=None):
    # 1) 시스템 세팅만 구성
    system_setting = GripperSystemSetting()
    system_setting.comport = ""
    system_setting.ip = "169.254.186.72"
    system_setting.port = 502
    system_setting.communication_mode = 0
    system_setting.control_mode = 1
    system_setting.read_timeout = 1000
    system_setting.slave_id = 0
    system_setting.baudrate = 0

    # 2) ROS2 init + 서비스 호출
    rclpy.init(args=args)
    node = Node("dg5fb_set_system_only")

    client = node.create_client(SetGripperSystem, "dg/set_gripper_system")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service dg/set_gripper_system not available, waiting...")

    req = SetGripperSystem.Request()
    req.setting = system_setting

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result is None:
        node.get_logger().error("Failed to call dg/set_gripper_system (no response).")
    else:
        print("SetGripperSystem Message:", result.result)

    # 3) 종료
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
