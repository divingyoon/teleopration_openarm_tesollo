#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


TOPICS = {
    0: "/dg5f_right/fingertip_1_broadcaster/wrench",
    1: "/dg5f_right/fingertip_2_broadcaster/wrench",
    2: "/dg5f_right/fingertip_3_broadcaster/wrench",
    3: "/dg5f_right/fingertip_4_broadcaster/wrench",
    4: "/dg5f_right/fingertip_5_broadcaster/wrench",
}


class FtPrinter(Node):
    def __init__(self):
        super().__init__("ft_printer_ros2_control")
        self.latest = {i: None for i in TOPICS.keys()}

        for i, topic in TOPICS.items():
            self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, ii=i: self.cb(ii, msg),
                10,
            )
            self.get_logger().info(f"subscribing: finger{i} <- {topic}")

        self.timer = self.create_timer(0.2, self.print_once)  # 5 Hz

    def cb(self, i: int, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.latest[i] = (f.x, f.y, f.z, t.x, t.y, t.z)

    def print_once(self):
        if all(v is None for v in self.latest.values()):
            self.get_logger().warn("no wrench received yet...")
            return

        best_i = None
        best_abs_fz = -1.0
        for i, v in self.latest.items():
            if v is None:
                continue
            fz = v[2]
            if abs(fz) > best_abs_fz:
                best_abs_fz = abs(fz)
                best_i = i

        lines = []
        lines.append("=" * 60)
        lines.append(f"[FT] strongest finger(by |Fz|): finger{best_i}  |Fz|={best_abs_fz:.2f}")
        for i in range(5):
            v = self.latest.get(i)
            if v is None:
                lines.append(f"  - finger{i}: (no data yet)")
                continue
            fx, fy, fz, tx, ty, tz = v
            prefix = "**" if i == best_i else "  "
            lines.append(
                f"{prefix}- finger{i}: "
                f"F=({fx:7.2f},{fy:7.2f},{fz:7.2f})  "
                f"T=({tx:7.2f},{ty:7.2f},{tz:7.2f})"
            )
        lines.append("=" * 60)
        self.get_logger().info("\n" + "\n".join(lines))


def main():
    rclpy.init()
    node = FtPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
