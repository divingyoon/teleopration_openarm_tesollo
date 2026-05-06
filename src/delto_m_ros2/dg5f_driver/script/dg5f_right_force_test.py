#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
import time
import math
from geometry_msgs.msg import WrenchStamped
import csv

def d2r(deg):
    return deg * math.pi / 180.0

class PIDControlTestAll(Node):

    def __init__(self):

        super().__init__('pid_control_test_all')

        self.joint_names = [
            "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
            "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
            "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
            "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
            "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"
        ]

        # 관절 목표값 publish 
        topic_name = '/dg5f_right/rj_dg_pospid/reference'

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            topic_name,
            10
        )

        # control frequency
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # trajectory timing
        self.t0 = time.time()

        # duration: 0도 -> 100도를 3초 동안 선형 이동
        self.move_time = 3.0

        # ===== force feedback settings =====
        self.current_angle_deg = 0.0

        self.phase = "approach"   # approach -> force_build -> hold
        self.contact_force_threshold = 1.0   # N, 접촉 판단용
        self.target_force = 3.0              # N
        self.force_tolerance = 0.2           # N, 2.8~3.2면 hold
        self.force_gain = 0.08               # 각도 보정 gain (작게 시작)
        self.max_angle_deg = 120.0           # 안전 상한
        self.min_angle_deg = 0.0

        self.contact_count = 0
        self.contact_count_required = 3   # 3 samples = 약 30ms

        # approach phase settings
        self.approach_speed_deg = 40.0       # deg/s
        self.approach_limit_deg = 100.0       # 접촉 안 나도 여기까지만 빠르게

        # latest wrench
        self.latest_force_xyz = None

        self.create_subscription(
            WrenchStamped,
            "/dg5f_right/fingertip_2_broadcaster/wrench",
            self.wrench_callback,
            10,
        )

        self.print_timer = self.create_timer(0.1, self.print_status)

        self.csv_file = open("/home/usr/ros2_ws/src/delto_m_ros2/real_force_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "phase", "angle_deg", "fz"])

        self.target_angle_deg = 120.0
        self.stop_after_hold_sec = 5.0
        self.stop_after_target_deg_sec = 5.0

        self.hold_start_time = None
        self.target_deg_reached_time = None
        self.stop_requested = False

    def wrench_callback(self, msg: WrenchStamped):
        f = msg.wrench.force
        self.latest_force_xyz = (f.x, f.y, f.z)

    def get_force_z(self):
        if self.latest_force_xyz is None:
            return None
        return self.latest_force_xyz[2]
    
    def timer_callback(self):

        elapsed = time.time() - self.t0
        fz_raw = self.get_force_z()
        fz_ctrl = None if fz_raw is None else -fz_raw

        if self.phase == "approach":
            self.current_angle_deg += self.approach_speed_deg * self.dt

            if fz_ctrl is not None and fz_ctrl >= self.contact_force_threshold:
                self.contact_count += 1
            else:
                self.contact_count = 0

            # 100도 이상 + 1.0N 이상이 연속으로 들어올 때만 force_build
            if (
                self.current_angle_deg >= self.approach_limit_deg
                and self.contact_count >= self.contact_count_required
            ):
                self.phase = "force_build"

        elif self.phase == "force_build":
            if fz_ctrl is None:
                self.current_angle_deg += 0.05
            else:
                error = self.target_force - fz_ctrl
                self.current_angle_deg += self.force_gain * error

                if fz_ctrl >= self.target_force:
                    self.phase = "hold"

        elif self.phase == "hold":
            if fz_ctrl is not None:
                error = self.target_force - fz_ctrl
                if abs(error) > self.force_tolerance:
                    self.current_angle_deg += 0.03 * error

        # target deg 도달 시각 기록
        if self.current_angle_deg >= self.target_angle_deg:
            if self.target_deg_reached_time is None:
                self.target_deg_reached_time = elapsed
        else:
            self.target_deg_reached_time = None

        # hold 진입 시각 기록
        if self.phase == "hold":
            if self.hold_start_time is None:
                self.hold_start_time = elapsed
        else:
            self.hold_start_time = None

        # 종료 조건 검사
        stop_reason = None

        if (
            self.hold_start_time is not None
            and (elapsed - self.hold_start_time) >= self.stop_after_hold_sec
        ):
            stop_reason = "hold phase >= 5s"

        if (
            self.target_deg_reached_time is not None
            and (elapsed - self.target_deg_reached_time) >= self.stop_after_target_deg_sec
        ):
            stop_reason = f"target angle >= {self.target_angle_deg} deg for 5s"

        if stop_reason is not None and not self.stop_requested:
            self.stop_requested = True
            self.get_logger().info(f"Stopping experiment: {stop_reason}")
            self.csv_file.flush()
            self.timer.cancel()
            self.print_timer.cancel()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return
        
        self.current_angle_deg = max(
            self.min_angle_deg,
            min(self.current_angle_deg, self.max_angle_deg)
        )

        q = d2r(self.current_angle_deg)

        position = [0.0] * 20
        position[5] = q   # rj_dg_2_2

        msg = MultiDOFCommand()
        msg.dof_names = self.joint_names
        msg.values = position
        msg.values_dot = [0.0] * 20

        self.publisher.publish(msg)

        self.csv_writer.writerow([elapsed, self.phase, self.current_angle_deg, "" if fz_raw is None else fz_raw])
        self.csv_file.flush()

    def print_status(self):
        elapsed = time.time() - self.t0
        fz_now = self.get_force_z()

        if fz_now is None:
            self.get_logger().info(
                f"time={elapsed:.3f}, phase={self.phase}, angle_deg={self.current_angle_deg:.2f}, fz=None"
            )
        else:
            self.get_logger().info(
                f"time={elapsed:.3f}, phase={self.phase}, angle_deg={self.current_angle_deg:.2f}, fz={fz_now:.4f}"
            )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PIDControlTestAll()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
