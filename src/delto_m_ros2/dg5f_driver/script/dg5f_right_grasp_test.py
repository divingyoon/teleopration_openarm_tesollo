#! /usr/bin/env python3
import time
import math
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import String
from sensor_msgs.msg import JointState  # JointState 임포트 추가


def d2r(deg):
    return deg * math.pi / 180.0


GRASP_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"
]

MOVE_TIME = 3.0
HOLD_TIME = 5.0
DT = 0.01
CONTACT_THRESHOLD = 0.5   # 우선 0.5N
FORCE_AXIS = "z"          # 1단계는 world z 기준으로 가정


class GraspAndForceCompare(Node):
    def __init__(self):
        super().__init__("grasp_and_force_compare")

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            "/dg5f_right/rj_dg_pospid/reference",
            10
        )

        self.joint_names = GRASP_JOINT_NAMES[:]
        # Initialize joint positions (0 rad for start)
        self.start_positions = [0.0] * len(self.joint_names)

        grasp_deg = {
            "rj_dg_1_1": 5.2,   "rj_dg_1_2": -100.5, "rj_dg_1_3": 41.1, "rj_dg_1_4": 10.3,
            "rj_dg_2_1": 5.9,   "rj_dg_2_2": 21.0,   "rj_dg_2_3": 57.0, "rj_dg_2_4": 34.7,
            "rj_dg_3_1": 4.7,   "rj_dg_3_2": 27.7,   "rj_dg_3_3": 60.2, "rj_dg_3_4": 38.7,
            "rj_dg_4_1": 2.2,   "rj_dg_4_2": 34.4,   "rj_dg_4_3": 53.8, "rj_dg_4_4": 40.4,
            "rj_dg_5_1": 3.8,   "rj_dg_5_2": 4.4,    "rj_dg_5_3": 44.4, "rj_dg_5_4": 45.4,
        }

        self.target_positions = [d2r(grasp_deg[name]) for name in self.joint_names]

        # Force threshold
        self.force_threshold = 0.5
        self.contact_count_threshold = 3

        self.contact_counter = {i: 0 for i in range(1, 6)}
        self.contact_confirmed = {i: False for i in range(1, 6)}
        self.contact_first_time = {i: None for i in range(1, 6)}

        # tip별 최신 wrench 저장
        self.latest_wrench = {i: None for i in range(1, 6)}
        
        # 최신 joint state 저장용 딕셔너리 추가
        self.latest_joint_states = {name: 0.0 for name in self.joint_names}

        self.tip_topics = {
            1: "/dg5f_right/fingertip_1_broadcaster/wrench",
            2: "/dg5f_right/fingertip_2_broadcaster/wrench",
            3: "/dg5f_right/fingertip_3_broadcaster/wrench",
            4: "/dg5f_right/fingertip_4_broadcaster/wrench",
            5: "/dg5f_right/fingertip_5_broadcaster/wrench",
        }

        # Wrench Subscribers
        self.subs = []
        for tip_idx, topic in self.tip_topics.items():
            sub = self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, idx=tip_idx: self.wrench_callback(idx, msg),
                10
            )
            self.subs.append(sub)

        # JointState Subscriber 추가 (실제 조인트 값 로깅 목적)
        # 참고: 로봇 설정에 따라 토픽 이름이 다를 수 있으니 필요시 수정하세요.
        self.joint_sub = self.create_subscription(
            JointState,
            "/dg5f_right/joint_states", 
            self.joint_state_callback,
            10
        )

        # grasp 시작 조건: right_pregrasp == "success"
        self.motion_started = False

        self.create_subscription(
            String,
            "right_pregrasp",
            self.pregrasp_callback,
            10
        )

        self.t0 = time.time()
        self.timer = self.create_timer(DT, self.timer_callback)
        self.finished = False

        self.move_time = MOVE_TIME
        self.hold_time = HOLD_TIME
        self.step_count = 0

        self.csv_file = open(
            "/home/usr/ros2_ws/src/delto_m_ros2/grasp_force_compare_real.csv",
            "w",
            newline=""
        )
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV 헤더에 조인트 이름들 동적으로 추가
        headers = [
            "time", "phase", "alpha",
            "fx_tip1", "fx_tip2", "fx_tip3", "fx_tip4", "fx_tip5",
            "fy_tip1", "fy_tip2", "fy_tip3", "fy_tip4", "fy_tip5",
            "fz_tip1", "fz_tip2", "fz_tip3", "fz_tip4", "fz_tip5",
            "contact_tip1", "contact_tip2", "contact_tip3", "contact_tip4", "contact_tip5"
        ] + self.joint_names
        
        self.csv_writer.writerow(headers)

        self.t0_sim = None

    def wrench_callback(self, tip_idx, msg: WrenchStamped):
        f = msg.wrench.force
        self.latest_wrench[tip_idx] = (f.x, f.y, f.z)

    def joint_state_callback(self, msg: JointState):
        """실제 로봇의 현재 조인트 상태를 캐싱합니다."""
        for i, name in enumerate(msg.name):
            if name in self.latest_joint_states:
                self.latest_joint_states[name] = msg.position[i]

    def get_force_z(self, tip_idx):
        if self.latest_wrench[tip_idx] is None:
            return None
        return self.latest_wrench[tip_idx][2]
    
    def get_axis_value(self, tip_idx):
        wrench = self.latest_wrench[tip_idx]
        if wrench is None:
            return None
        fx, fy, fz = wrench
        if FORCE_AXIS == "x":
            return fx
        elif FORCE_AXIS == "y":
            return fy
        return fz
    
    def pregrasp_callback(self, msg: String):
        data = msg.data.strip()

        if data == "success" and not self.motion_started:
            self.motion_started = True
            self.t0_sim = time.time()  # success 신호를 받은 시점에서 시뮬레이션 시간 시작

            # contact 관련 상태도 시작 시점에 맞춰 초기화
            self.contact_counter = {i: 0 for i in range(1, 6)}
            self.contact_confirmed = {i: False for i in range(1, 6)}
            self.contact_first_time = {i: None for i in range(1, 6)}
            self.step_count = 0

            self.get_logger().info("Received 'success' from right_pregrasp. Starting grasp motion.")
        elif self.motion_started:
            self.get_logger().info(f"Motion already started. Ignored message: '{data}'")
        else:
            self.get_logger().info(f"Ignored message on right_pregrasp: '{data}'")

    def timer_callback(self):
        # --- 수정된 부분: success 신호 대기 중일 때 기본 0 자세 유지 ---
        if not self.motion_started:
            msg = MultiDOFCommand()
            msg.dof_names = self.joint_names
            msg.values = self.start_positions  # [0.0, 0.0, ...] 기본 0 자세
            msg.values_dot = [0.0] * len(self.joint_names)
            self.publisher.publish(msg)
            return
        # -----------------------------------------------------------------

        if self.t0_sim is None:  # success 신호를 받기 전에 timer_callback이 호출되면 종료
            self.t0_sim = time.time()  # 초기화 시점에서 시간을 시작할 수 있습니다.
            return

        # 이후 정상적으로 시간 계산
        elapsed = time.time() - self.t0_sim

        if elapsed <= self.move_time:
            alpha = elapsed / self.move_time
            phase = "move"
        else:
            alpha = 1.0
            phase = "hold"

        # Linear interpolation for joint positions
        position = []
        for q0, q1 in zip(self.start_positions, self.target_positions):
            q = q0 + alpha * (q1 - q0)
            position.append(q)

        msg = MultiDOFCommand()
        msg.dof_names = self.joint_names
        msg.values = position
        msg.values_dot = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)

        # tip별 fx, fy, fz 수집 + contact 판정
        fx_list, fy_list, fz_list = [], [], []
        contact_flags = []

        for tip_idx in range(1, 6):
            wrench = self.latest_wrench[tip_idx]
            
            if wrench is None:
                fx_list.append("")
                fy_list.append("")
                fz_list.append("")
                axis_val = None
            else:
                fx, fy, fz = wrench
                fx_list.append(fx)
                fy_list.append(fy)
                fz_list.append(fz)
                
                # Contact 판정을 위한 대표 축 값 선택 (기존 로직 유지)
                if FORCE_AXIS == "x":
                    axis_val = fx
                elif FORCE_AXIS == "y":
                    axis_val = fy
                else:
                    axis_val = fz

            if axis_val is not None and abs(axis_val) > self.force_threshold:
                self.contact_counter[tip_idx] += 1
            else:
                self.contact_counter[tip_idx] = 0

            if (
                self.contact_counter[tip_idx] >= self.contact_count_threshold
                and not self.contact_confirmed[tip_idx]
            ):
                self.contact_confirmed[tip_idx] = True
                self.contact_first_time[tip_idx] = elapsed
                self.get_logger().info(
                    f"[CONTACT] tip{tip_idx} confirmed at t={elapsed:.3f}s, {FORCE_AXIS}={axis_val:.3f}"
                )

            contact_flags.append(1 if self.contact_confirmed[tip_idx] else 0)

        # 현재 실제 조인트 값 스냅샷 생성
        current_joint_vals = [self.latest_joint_states[name] for name in self.joint_names]

        # CSV 저장 (success 이후의 데이터만 기록)
        row_data = (
            [elapsed, phase, alpha] + 
            fx_list + 
            fy_list + 
            fz_list + 
            contact_flags + 
            current_joint_vals
        )
        
        self.csv_writer.writerow(row_data)
        self.csv_file.flush()

        # 원래처럼 총 8초 후 종료
        if elapsed >= (self.move_time + self.hold_time) and not self.finished:
            self.finished = True
            self.get_logger().info("Finished grasp motion. Shutting down.")
            for tip_idx in range(1, 6):
                self.get_logger().info(
                    f"tip{tip_idx} first_contact_time = {self.contact_first_time[tip_idx]}"
                )
            self.timer.cancel()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        self.step_count += 1

    def finish(self):
        if self.finished:
            return

        self.finished = True
        self.get_logger().info("Finished grasp motion.")
        for tip_idx in range(1, 6):
            self.get_logger().info(
                f"tip{tip_idx} first_contact_time = {self.contact_first_time[tip_idx]}"
            )

        self.timer.cancel()
        self.csv_file.close()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GraspAndForceCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.finish()


if __name__ == "__main__":
    main()