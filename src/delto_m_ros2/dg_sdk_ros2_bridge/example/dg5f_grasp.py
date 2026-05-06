#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# dg_msgs의 서비스들을 임포트합니다. 
# 실제 환경에 따라 서비스 명칭이 조금 다를 수 있으니 ros2 service list로 확인하세요.
from dg_msgs.srv import (
    SetGripperSystem, 
    ConnectToGripper, 
    SetGripperOption, 
    SystemStart, 
    SetGraspData, 
    StartGraspMotion
)
from dg_msgs.msg import GripperSystemSetting, GripperSetting

class DG5F_GraspTester(Node):
    def __init__(self):
        super().__init__('dg5f_grasp_tester')

        # 1. 서비스 클라이언트 생성
        self.set_sys_client = self.create_client(SetGripperSystem, 'dg/set_gripper_system')
        self.connect_client = self.create_client(ConnectToGripper, 'dg/connect_to_gripper')
        self.set_opt_client = self.create_client(SetGripperOption, 'dg/set_gripper_option')
        self.start_sys_client = self.create_client(SystemStart, 'dg/system_start')
        self.set_grasp_client = self.create_client(SetGraspData, 'dg/set_grasp_data')
        self.motion_client = self.create_client(StartGraspMotion, 'dg/start_grasp_motion')

        self.get_logger().info('DG-5F Grasp Tester 노드가 시작되었습니다.')

    def call_service(self, client, request):
        """서비스 호출을 위한 헬퍼 함수"""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{client.srv_name} 서비스 대기 중...')
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_test(self):
        # [Step 1] 성공 코드(dg5fb.py)와 동일한 파라미터 적용
        sys_req = SetGripperSystem.Request()
        sys_req.setting.ip = "169.254.186.72"
        sys_req.setting.port = 502
        sys_req.setting.communication_mode = 0 
        sys_req.setting.control_mode = 1        # 하드웨어 설정에 맞춤
        sys_req.setting.read_timeout = 1000     # 성공 코드 값 적용
        self.call_service(self.set_sys_client, sys_req)
        self.get_logger().info('시스템 설정 완료')
        time.sleep(1.0)

        # [Step 2] 연결 수립
        self.call_service(self.connect_client, ConnectToGripper.Request())
        self.get_logger().info('그리퍼 연결 중...')
        time.sleep(2.0) # Connected 로그를 확인하기 위한 충분한 시간

        # [Step 3] 옵션 설정 시 '수신 데이터 타입' 명시 (매우 중요)
        opt_req = SetGripperOption.Request()
        opt_req.setting.model = 24354           # DG-5F Right
        opt_req.setting.joint_count = 20
        opt_req.setting.finger_count = 5
        # 1: Joint, 2: Current, 5: FT Sensor 데이터를 받겠다고 선언
        opt_req.setting.received_data_type = [1, 2, 0, 0, 5, 0] 
        self.call_service(self.set_opt_client, opt_req)
        self.get_logger().info('모델 및 데이터 타입 설정 완료')
        time.sleep(1.0)

        # [Step 4] 시스템 가동 시작
        self.call_service(self.start_sys_client, SystemStart.Request())
        self.get_logger().info('시스템 가동(Start) 명령 전송')
        time.sleep(2.0) # 가동 후 안정화 대기

        # [Step 5] 다양한 Grasp Mode 테스트 루프
        # 21: 5지 기본, 24: 엄지-검지 핀치, 30: 5지 평행, 31: 5지 감싸쥐기
        test_modes = {
            21: "5-Finger Basic",
            24: "2-Finger Pinch (1 & 2)",
            30: "5-Finger Parallel",
            31: "5-Finger Envelop"
        }

        for mode_val, mode_name in test_modes.items():
            self.get_logger().info(f'>>> 테스트 모드 실행: {mode_name} ({mode_val})')

            # 그랩 데이터 설정
            grasp_req = SetGraspData.Request()
            grasp_req.grasp_mode = mode_val
            grasp_req.grasp_force = 5.0  # 그랩 힘 설정
            grasp_req.grasp_option = 0    # 옵션 없음
            grasp_req.smooth_grasping = 1 # 부드러운 그랩
            self.call_service(self.set_grasp_client, grasp_req)

            # 그랩 동작 시작
            motion_req = StartGraspMotion.Request()
            motion_req.is_grasp = 1       # 1: 그랩, 0: 해제
            self.call_service(self.motion_client, motion_req)
            
            time.sleep(4.0)  # 동작 관찰을 위한 대기

            # 그랩 해제 (Open)
            motion_req.is_grasp = 0
            self.call_service(self.motion_client, motion_req)
            self.get_logger().info(f'{mode_name} 테스트 완료 및 초기화')
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    tester = DG5F_GraspTester()
    try:
        tester.run_test()
    except Exception as e:
        tester.get_logger().error(f'에러 발생: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()