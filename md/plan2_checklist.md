# Plan2 Checklist: Tesollo Right Hand Setting and Logging

## 1. 사전 조건

- [ ] `plan1_jointvel_torque_right_arm.md` 구현이 완료되었다.
- [ ] `colcon build --packages-select openarm_teleop`가 성공한다.
- [ ] right teleoperation 실행 시 `/openarm/right/joint_states`가 publish된다.
- [ ] `/openarm/right/joint_states`의 `position`, `velocity`, `effort` 배열 길이가 `name` 배열 길이와 같다.
- [ ] right follower에는 OpenArm gripper 대신 Tesollo DG5F right hand가 장착되어 있다.
- [ ] Tesollo DG5F right hand의 IP와 port를 확인했다.
- [ ] Tesollo fingertip sensor 사용 여부를 확인했다.

## 2. Tesollo Driver 확인

- [ ] 다음 명령으로 Tesollo right driver가 실행된다.

```bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_driver.launch.py fingertip_sensor:=true
```

- [ ] 필요 시 IP와 port를 명시해 실행한다.

```bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_driver.launch.py \
  delto_ip:=169.254.186.72 \
  delto_port:=502 \
  fingertip_sensor:=true
```

- [ ] `/dg5f_right/joint_states`가 publish된다.
- [ ] `/dg5f_right/fingertip_1_broadcaster/wrench`가 publish된다.
- [ ] `/dg5f_right/fingertip_2_broadcaster/wrench`가 publish된다.
- [ ] `/dg5f_right/fingertip_3_broadcaster/wrench`가 publish된다.
- [ ] `/dg5f_right/fingertip_4_broadcaster/wrench`가 publish된다.
- [ ] `/dg5f_right/fingertip_5_broadcaster/wrench`가 publish된다.

확인 명령:

```bash
ROS_DOMAIN_ID=126 ros2 topic list | grep dg5f_right
ROS_DOMAIN_ID=126 ros2 topic echo /dg5f_right/joint_states --once
ROS_DOMAIN_ID=126 ros2 topic echo /dg5f_right/fingertip_1_broadcaster/wrench --once
```

## 3. OpenArm Leader Gripper Topic 구현

> **참고**: 기존 파일 유지 원칙에 따라 `openarm_unilateral_control_v2.cpp`에 구현했다.

- [x] `control/openarm_unilateral_control_v2.cpp`에 leader gripper publisher를 추가했다.
- [x] publisher topic은 `/openarm/right/leader/gripper_state`로 한다.
- [x] message type은 `sensor_msgs/msg/JointState`로 한다.
- [x] `arm_side == "right_arm"`일 때만 publish한다.
- [x] `name`에는 `right_leader_gripper_joint_0`을 넣는다.
- [x] `position`에는 leader right gripper position을 넣는다.
- [x] `velocity`에는 leader right gripper velocity를 넣는다.
- [x] `effort`에는 leader right gripper effort/torque를 넣는다.
- [x] `name`, `position`, `velocity`, `effort` 배열 길이가 항상 같다.

확인 명령:

```bash
ROS_DOMAIN_ID=126 ros2 topic echo /openarm/right/leader/gripper_state
```

완료 기준:

- [ ] leader right gripper를 움직이면 `/openarm/right/leader/gripper_state.position[0]` 값이 연속적으로 변한다.

## 4. Tesollo Bridge Node 구현

- [x] `src/openarm_teleop/script/tesollo_right_hand_setting_bridge.py`를 추가한다.
- [x] node 이름은 `tesollo_right_hand_setting_bridge`로 한다.
- [x] subscribe topic은 `/openarm/right/leader/gripper_state`로 한다.
- [x] publish topic은 `/dg5f_right/rj_dg_pospid/reference`로 한다.
- [x] publish message type은 `control_msgs/msg/MultiDOFCommand`로 한다.
- [x] publish 주기는 기본 `100 Hz`로 한다.
- [x] leader gripper position을 `alpha`로 normalize한다.
- [x] `alpha`는 항상 `[0.0, 1.0]`로 clamp한다.
- [x] `invert_input` 파라미터가 true이면 `alpha = 1.0 - alpha`를 적용한다.
- [x] 입력 timeout 기본값은 `0.5 s`로 한다.
- [x] timeout 시 마지막 valid command를 유지한다.

필수 파라미터:

- [x] `input_topic`
- [x] `output_topic`
- [x] `publish_rate_hz`
- [x] `leader_open_position`
- [x] `leader_grasp_position`
- [x] `invert_input`
- [x] `input_timeout_sec`
- [x] `hold_last_on_timeout`

오류 처리:

- [x] `leader_open_position == leader_grasp_position`이면 error log를 출력하고 publish를 중단한다.
- [x] 입력 `JointState.position`이 비어 있으면 command를 갱신하지 않는다.
- [x] 입력값이 NaN 또는 inf이면 command를 갱신하지 않는다.

## 5. Tesollo Joint Target 구현

- [x] Tesollo joint name 20개를 고정 순서로 정의한다.
- [x] 순서는 `dg5f_right_controller.yaml`의 right joint 순서와 동일하다.
- [x] 초기 자세는 모든 joint `0.0 rad`로 한다.
- [x] grasp 자세는 `dg5f_right_grasp_test.py`의 `grasp_deg` 값을 사용한다.
- [x] grasp 자세는 degree에서 radian으로 변환한다.
- [x] target 계산식은 `q = q_initial + alpha * (q_grasp - q_initial)`로 한다.
- [x] command의 `dof_names`, `values`, `values_dot` 길이는 항상 20이다.
- [x] `values_dot`는 v1에서 모두 `0.0`으로 publish한다.

확인 명령:

```bash
ROS_DOMAIN_ID=126 ros2 topic echo /dg5f_right/rj_dg_pospid/reference
```

완료 기준:

- [ ] leader gripper open 위치에서 Tesollo target은 초기 자세에 가깝다.
- [ ] leader gripper grasp 위치에서 Tesollo target은 grasp 자세에 가깝다.
- [ ] leader gripper 중간 위치에서 Tesollo target은 초기 자세와 grasp 자세 사이에 있다.

## 6. Build 설정

- [x] `src/openarm_teleop/CMakeLists.txt`에 bridge script install rule을 추가한다.

```cmake
install(
  PROGRAMS
    script/tesollo_right_hand_setting_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
```

- [x] `src/openarm_teleop/package.xml`에 필요한 의존성을 추가한다.

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>control_msgs</depend>
```

- [x] `sensor_msgs` 의존성은 기존에 있으므로 중복 추가하지 않는다.

Build 명령:

```bash
cd /home/user/Desktop/ros2_ws
colcon build --packages-select openarm_teleop dg5f_driver delto_hardware
source install/setup.bash
```

완료 기준:

- [x] build가 성공한다.
- [ ] `ros2 run openarm_teleop tesollo_right_hand_setting_bridge`가 실행된다. ← 실행 테스트 필요

## 7. 통합 실행

Terminal 1: CAN 설정

- [ ] 다음 명령을 실행한다.

```bash
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can3 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan-4-arms -fd
```

Terminal 2: OpenArm right teleoperation

- [ ] 다음 명령을 실행한다.

```bash
cd /home/user/Desktop/ros2_ws/src/openarm_teleop
./script/launch_unilateral.sh right_arm can0 can2
```

Terminal 3: Tesollo right hand

- [ ] 다음 명령을 실행한다.

```bash
cd /home/user/Desktop/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_driver.launch.py fingertip_sensor:=true
```

Terminal 4: Tesollo bridge

- [ ] 다음 명령을 실행한다.

```bash
cd /home/user/Desktop/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=126 ros2 run openarm_teleop tesollo_right_hand_setting_bridge
```

Terminal 5: rosbag record

- [ ] 다음 명령을 실행한다.

```bash
ROS_DOMAIN_ID=126 ros2 bag record -s sqlite3 --max-cache-size 100000000 \
  /output \
  /openarm/right/joint_states \
  /openarm/right/leader/gripper_state \
  /dg5f_right/joint_states \
  /dg5f_right/fingertip_1_broadcaster/wrench \
  /dg5f_right/fingertip_2_broadcaster/wrench \
  /dg5f_right/fingertip_3_broadcaster/wrench \
  /dg5f_right/fingertip_4_broadcaster/wrench \
  /dg5f_right/fingertip_5_broadcaster/wrench \
  /color/image_raw/compressed \
  /color/camera_info \
  /aligned_depth_to_color/image_raw \
  /tf
```

## 8. Teleoperation 중 확인

- [ ] leader right gripper를 천천히 open에서 grasp 방향으로 움직인다.
- [ ] Tesollo hand가 튀지 않고 연속적으로 움직인다.
- [ ] leader right gripper를 중간에서 멈추면 Tesollo hand도 중간 자세를 유지한다.
- [ ] leader right gripper를 open 방향으로 되돌리면 Tesollo hand가 초기 자세로 돌아간다.
- [ ] fingertip 접촉 시 wrench 값이 변한다.
- [ ] OpenArm right arm 움직임과 Tesollo hand 움직임이 동시에 bag에 기록된다.

## 9. Rosbag 검증

- [ ] 기록 완료 후 bag 정보를 확인한다.

```bash
ros2 bag info <bag_dir>
```

- [ ] 다음 토픽이 bag에 포함되어 있다.

```text
/openarm/right/joint_states
/openarm/right/leader/gripper_state
/dg5f_right/joint_states
/dg5f_right/fingertip_1_broadcaster/wrench
/dg5f_right/fingertip_2_broadcaster/wrench
/dg5f_right/fingertip_3_broadcaster/wrench
/dg5f_right/fingertip_4_broadcaster/wrench
/dg5f_right/fingertip_5_broadcaster/wrench
```

- [ ] `/dg5f_right/joint_states`에 20개 joint name이 들어 있다.
- [ ] `/dg5f_right/joint_states.position` 길이가 20이다.
- [ ] `/dg5f_right/joint_states.velocity` 길이가 20이다.
- [ ] `/dg5f_right/joint_states.effort` 길이가 20이다.
- [ ] 각 fingertip wrench 토픽에서 `force.x`, `force.y`, `force.z`가 기록된다.
- [ ] 각 fingertip wrench 토픽에서 `torque.x`, `torque.y`, `torque.z`가 기록된다.

## 10. Acceptance Criteria

- [ ] `/openarm/right/leader/gripper_state`가 publish된다.
- [ ] `/dg5f_right/joint_states`가 publish된다.
- [ ] 5개 fingertip wrench topic이 publish된다.
- [ ] leader right gripper 입력으로 Tesollo right hand가 초기 자세에서 grasp 자세까지 연속 이동한다.
- [ ] Tesollo command 배열 길이는 항상 20이다.
- [ ] OpenArm right arm, leader gripper input, Tesollo joint state, Tesollo fingertip wrench가 하나의 rosbag에 기록된다.
- [ ] bag replay로 Tesollo joint position, velocity, effort/current, fingertip `fx, fy, fz, tx, ty, tz`를 확인할 수 있다.

## 11. 남은 확인 사항

- [ ] `/dg5f_right/joint_states.effort`의 실제 단위가 current인지 torque인지 확인한다.
- [ ] leader gripper open/grasp 실제 min/max 값을 실측해 `leader_open_position`, `leader_grasp_position` 기본값을 보정한다.
- [ ] Tesollo grasp target이 실제 object/task에 맞는지 확인한다.
- [ ] fingertip sensor frame 방향과 부호를 실험 데이터로 확인한다.
