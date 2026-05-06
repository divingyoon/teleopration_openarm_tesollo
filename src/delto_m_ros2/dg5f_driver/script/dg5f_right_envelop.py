#! /usr/bin/env python3
import time
import math
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import String
from sensor_msgs.msg import JointState


def d2r(deg):
    return deg * math.pi / 180.0

# 손가락별 조인트 매핑
FINGER_JOINTS = {
    1: ["rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4"],
    2: ["rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4"],
    3: ["rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4"],
    4: ["rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4"],
    5: ["rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4"],
}

ALL_JOINT_NAMES = []
for i in range(1, 6):
    ALL_JOINT_NAMES.extend(FINGER_JOINTS[i])

DT = 0.01

# --- 실시간 힘 제어 파라미터 ---
TARGET_FORCE = 3.0       # 목표 그랩 힘 (N) - 필요시 수정하세요.
FORCE_AXIS = "z"         # 힘 측정 기준 축
CONTACT_THRESHOLD = 0.3  # 이 힘 이상이면 접촉으로 간주 (N)
KP_FORCE = 0.05          # 힘 제어 비례 게인 (P-Gain)
APPROACH_SPEED = 0.3     # 허공에서 손가락이 닫히는 속도 (Alpha/sec)
# ------------------------------

class RealTimeForceGrasp(Node):
    def __init__(self):
        super().__init__("realtime_force_grasp")

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            "/dg5f_right/rj_dg_pospid/reference",
            10
        )

        self.joint_names = ALL_JOINT_NAMES
        
        # 각 손가락의 현재 '닫힘 정도' (0.0: 완전 폄, 1.0: 완전 쥠)
        self.finger_alphas = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}

        # 시작 자세 (전체 0.0)
        self.start_positions = {name: 0.0 for name in self.joint_names}

        # 감싸쥐기(Enveloping) 최대 목표 자세 (기존 코드의 grasp_deg 활용)
        grasp_deg = {
            "rj_dg_1_1": 10.0,   "rj_dg_1_2": -110.0, "rj_dg_1_3": 45.0, "rj_dg_1_4": 25.0,
            "rj_dg_2_1": 10.0,   "rj_dg_2_2": 25.0,   "rj_dg_2_3": 60.0, "rj_dg_2_4": 50.0,
            "rj_dg_3_1": 5.0,   "rj_dg_3_2": 30.0,   "rj_dg_3_3": 65.0, "rj_dg_3_4": 50.0,
            "rj_dg_4_1": 5.0,   "rj_dg_4_2": 35.0,   "rj_dg_4_3": 55.0, "rj_dg_4_4": 55.0,
            "rj_dg_5_1": 5.0,   "rj_dg_5_2": 5.0,    "rj_dg_5_3": 50.0, "rj_dg_5_4": 60.0,
        }
        self.target_positions = {name: d2r(val) for name, val in grasp_deg.items()}

        # 센서 데이터 캐싱
        self.latest_wrench = {i: None for i in range(1, 6)}
        self.latest_joint_states = {name: 0.0 for name in self.joint_names}

        self.tip_topics = {
            1: "/dg5f_right/fingertip_1_broadcaster/wrench",
            2: "/dg5f_right/fingertip_2_broadcaster/wrench",
            3: "/dg5f_right/fingertip_3_broadcaster/wrench",
            4: "/dg5f_right/fingertip_4_broadcaster/wrench",
            5: "/dg5f_right/fingertip_5_broadcaster/wrench",
        }

        self.subs = []
        for tip_idx, topic in self.tip_topics.items():
            sub = self.create_subscription(
                WrenchStamped,
                topic,
                lambda msg, idx=tip_idx: self.wrench_callback(idx, msg),
                10
            )
            self.subs.append(sub)

        self.joint_sub = self.create_subscription(
            JointState,
            "/dg5f_right/joint_states", 
            self.joint_state_callback,
            10
        )

        self.motion_started = False
        self.create_subscription(String, "right_pregrasp", self.pregrasp_callback, 10)

        self.timer = self.create_timer(DT, self.timer_callback)
        
        # CSV 로깅 설정
        self.csv_file = open("realtime_grasp_force_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        headers = ["time"] + \
                  [f"alpha_tip{i}" for i in range(1, 6)] + \
                  [f"force_tip{i}" for i in range(1, 6)] + \
                  self.joint_names
        self.csv_writer.writerow(headers)

        self.t0_sim = None
        self.get_logger().info("Real-time Force Grasp Node Initialized. Waiting for 'success' signal...")

    def wrench_callback(self, tip_idx, msg: WrenchStamped):
        f = msg.wrench.force
        self.latest_wrench[tip_idx] = (f.x, f.y, f.z)

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.latest_joint_states:
                self.latest_joint_states[name] = msg.position[i]

    def get_axis_force(self, tip_idx):
        wrench = self.latest_wrench[tip_idx]
        if wrench is None:
            return 0.0
        fx, fy, fz = wrench
        if FORCE_AXIS == "x": return abs(fx)
        elif FORCE_AXIS == "y": return abs(fy)
        return abs(fz)

    def pregrasp_callback(self, msg: String):
        data = msg.data.strip()
        if data == "success" and not self.motion_started:
            self.motion_started = True
            self.t0_sim = time.time()
            self.get_logger().info("Received 'success'. Starting Real-time Force Control Grasp.")

    def timer_callback(self):
        if not self.motion_started:
            # 대기 상태: 기본 0 자세 유지
            msg = MultiDOFCommand()
            msg.dof_names = self.joint_names
            msg.values = [self.start_positions[name] for name in self.joint_names]
            msg.values_dot = [0.0] * len(self.joint_names)
            self.publisher.publish(msg)
            return

        elapsed = time.time() - self.t0_sim
        current_forces = {}

        # 1. 각 손가락별로 독립적인 위치 업데이트 (핵심 알고리즘)
        for i in range(1, 6):
            f_val = self.get_axis_force(i)
            current_forces[i] = f_val

            if f_val < CONTACT_THRESHOLD:
                # 허공 상태: 접촉할 때까지 일정한 속도로 손가락을 닫음
                self.finger_alphas[i] += APPROACH_SPEED * DT
            else:
                # 접촉 상태: 목표 힘과의 오차를 계산하여 위치(Alpha)를 제어 (Admittance Control)
                force_error = TARGET_FORCE - f_val
                # 오차가 양수면 더 닫고, 음수(힘이 너무 셈)면 열어서 힘을 뺌
                self.finger_alphas[i] += KP_FORCE * force_error * DT

            # Alpha 값은 0(완전 폄)과 1(설정된 최대 그랩 자세) 사이로 제한
            self.finger_alphas[i] = max(0.0, min(1.0, self.finger_alphas[i]))

        # 2. 계산된 Alpha를 바탕으로 각 관절의 위치(Position) 결정
        position_cmds = []
        for name in self.joint_names:
            # 현재 관절이 속한 손가락의 번호를 찾음
            finger_idx = int(name.split("_")[2])
            alpha = self.finger_alphas[finger_idx]
            
            q0 = self.start_positions[name]
            q1 = self.target_positions[name]
            q_cmd = q0 + alpha * (q1 - q0)
            position_cmds.append(q_cmd)

        # 3. 로봇으로 명령 전송
        msg = MultiDOFCommand()
        msg.dof_names = self.joint_names
        msg.values = position_cmds
        msg.values_dot = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)

        # 4. 데이터 로깅
        current_joint_vals = [self.latest_joint_states[name] for name in self.joint_names]
        row_data = [elapsed] + \
                   [self.finger_alphas[i] for i in range(1, 6)] + \
                   [current_forces[i] for i in range(1, 6)] + \
                   current_joint_vals
        self.csv_writer.writerow(row_data)

    def finish(self):
        self.get_logger().info("Shutting down Real-time Force Grasp Node.")
        self.timer.cancel()
        self.csv_file.close()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeForceGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.finish()

if __name__ == "__main__":
    main()