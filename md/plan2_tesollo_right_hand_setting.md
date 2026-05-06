# Plan2: Tesollo Right Hand Setting and Logging

## 1. 목적

`plan1_jointvel_torque_right_arm.md` 구현 이후, right follower arm에 기존 OpenArm gripper 대신 장착된 Tesollo DG5F right hand를 teleoperation 흐름에 연결한다.

목표는 다음과 같다.

- OpenArm leader right arm gripper 입력을 연속 제어 입력으로 사용한다.
- Tesollo right hand를 초기 자세에서 grasp 자세까지 연속적으로 움직인다.
- teleoperation 중 Tesollo hand의 joint state, velocity, effort/current, fingertip sensor force/torque를 rosbag에 함께 기록한다.

이 문서는 실제 구현을 위한 설계 기준이다.

## 2. 전제 조건

Plan2는 Plan1이 먼저 구현되어 있다는 전제로 작성한다.

Plan1 구현 후 기대되는 상태:

- `/openarm/right/joint_states`가 publish된다.
- right follower arm의 position, velocity, effort가 `sensor_msgs/msg/JointState`에 포함된다.
- right side 실행 시 OpenArm right arm 데이터와 left arm 데이터가 분리된다.

Plan2에서 추가로 필요한 상태:

- OpenArm leader right gripper 상태가 별도 토픽으로 publish되어야 한다.
- Tesollo DG5F right hand driver가 `fingertip_sensor:=true`로 실행되어야 한다.
- rosbag 기록 명령에 Tesollo joint state와 fingertip wrench 토픽이 포함되어야 한다.

## 3. 참고 소스

Tesollo 관련 구현은 다음 소스를 기준으로 한다.

| 항목 | 파일 |
|---|---|
| DG5F right launch | `src/delto_m_ros2/dg5f_driver/launch/dg5f_right_driver.launch.py` |
| DG5F right controller config | `src/delto_m_ros2/dg5f_driver/config/dg5f_right_controller.yaml` |
| DG5F right fingertip broadcaster config | `src/delto_m_ros2/dg5f_driver/config/dg5f_right_ft_broadcaster.yaml` |
| DG5F grasp target example | `src/delto_m_ros2/dg5f_driver/script/dg5f_right_grasp_test.py` |
| DG5F hardware state export | `src/delto_m_ros2/delto_hardware/src/system_interface.cpp` |

현재 확인된 구조:

- DG5F right hand namespace는 `/dg5f_right`이다.
- Tesollo joint state topic은 `/dg5f_right/joint_states`이다.
- Tesollo control topic은 `/dg5f_right/rj_dg_pospid/reference`이다.
- control message type은 `control_msgs/msg/MultiDOFCommand`이다.
- fingertip sensor는 5개 `geometry_msgs/msg/WrenchStamped` 토픽으로 publish된다.

## 4. 실행 구조

Plan2의 전체 실행 구조는 다음과 같다.

```text
OpenArm leader right gripper
        |
        v
/openarm/right/leader/gripper_state
        |
        v
tesollo_right_hand_setting_bridge
        |
        v
/dg5f_right/rj_dg_pospid/reference
        |
        v
Tesollo DG5F right hand
        |
        +--> /dg5f_right/joint_states
        +--> /dg5f_right/fingertip_1_broadcaster/wrench
        +--> /dg5f_right/fingertip_2_broadcaster/wrench
        +--> /dg5f_right/fingertip_3_broadcaster/wrench
        +--> /dg5f_right/fingertip_4_broadcaster/wrench
        +--> /dg5f_right/fingertip_5_broadcaster/wrench
```

## 5. Tesollo Driver 실행

Tesollo right hand는 다음 launch로 실행한다.

```bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_driver.launch.py fingertip_sensor:=true
```

필요 시 IP와 port를 명시한다.

```bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_driver.launch.py \
  delto_ip:=169.254.186.72 \
  delto_port:=502 \
  fingertip_sensor:=true
```

실행 후 확인해야 하는 토픽:

```text
/dg5f_right/joint_states
/dg5f_right/fingertip_1_broadcaster/wrench
/dg5f_right/fingertip_2_broadcaster/wrench
/dg5f_right/fingertip_3_broadcaster/wrench
/dg5f_right/fingertip_4_broadcaster/wrench
/dg5f_right/fingertip_5_broadcaster/wrench
```

## 6. OpenArm Leader Gripper State Topic

Tesollo bridge가 OpenArm leader right gripper 입력을 사용할 수 있도록 OpenArm teleop 쪽에서 leader gripper 상태를 별도 publish한다.

권장 토픽:

```text
/openarm/right/leader/gripper_state
```

메시지 타입:

```text
sensor_msgs/msg/JointState
```

필드 사용:

| 필드 | 값 |
|---|---|
| `header.stamp` | publish 시점 ROS time |
| `name` | `right_leader_gripper_joint_0` |
| `position` | leader right gripper position |
| `velocity` | leader right gripper velocity |
| `effort` | leader right gripper effort/torque |

구현 위치:

```text
src/openarm_teleop/control/openarm_unilateral_control.cpp
```

권장 구현:

- `AdminThread`에 leader gripper publisher를 추가한다.
- `arm_side == "right_arm"`일 때만 `/openarm/right/leader/gripper_state`를 publish한다.
- `leader_state_->hand_state().get_all_responses()`에서 leader gripper response를 읽는다.
- 현재 OpenArm gripper motor가 1개이므로 배열 길이는 1개를 기본으로 한다.

주의:

- follower에는 Tesollo hand가 장착되어 있으므로, Plan2에서 OpenArm follower gripper 제어는 사용하지 않는다.
- OpenArm leader gripper는 입력 장치 역할만 한다.

## 7. Tesollo Bridge Node 설계

신규 bridge node를 추가한다.

권장 파일:

```text
src/openarm_teleop/script/tesollo_right_hand_setting_bridge.py
```

node 이름:

```text
tesollo_right_hand_setting_bridge
```

subscribe:

```text
/openarm/right/leader/gripper_state
```

publish:

```text
/dg5f_right/rj_dg_pospid/reference
```

publish type:

```text
control_msgs/msg/MultiDOFCommand
```

주기:

```text
100 Hz
```

기본 동작:

1. leader gripper position을 읽는다.
2. position을 `alpha`로 normalize한다.
3. `alpha`를 `[0.0, 1.0]` 범위로 clamp한다.
4. Tesollo joint target을 `q = q_initial + alpha * (q_grasp - q_initial)`로 계산한다.
5. `/dg5f_right/rj_dg_pospid/reference`에 publish한다.

입력 timeout:

- leader gripper 입력이 일정 시간 들어오지 않으면 마지막 valid command를 유지한다.
- 권장 timeout 값은 `0.5 s`이다.
- timeout 중에는 새 target을 만들지 않고 마지막 target을 재publish한다.

## 8. Bridge Parameters

bridge node는 다음 파라미터를 가진다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `input_topic` | `/openarm/right/leader/gripper_state` | leader gripper 입력 토픽 |
| `output_topic` | `/dg5f_right/rj_dg_pospid/reference` | Tesollo command 출력 토픽 |
| `publish_rate_hz` | `100.0` | command publish 주기 |
| `leader_open_position` | `0.0` | leader gripper 완전 open 기준값 |
| `leader_grasp_position` | `1.0` | leader gripper 완전 grasp 기준값 |
| `invert_input` | `false` | leader 입력 방향 반전 여부 |
| `input_timeout_sec` | `0.5` | 입력 timeout |
| `hold_last_on_timeout` | `true` | timeout 때 마지막 command 유지 |

`alpha` 계산:

```text
raw_alpha = (leader_position - leader_open_position) /
            (leader_grasp_position - leader_open_position)
alpha = clamp(raw_alpha, 0.0, 1.0)
```

`invert_input == true`이면 다음을 적용한다.

```text
alpha = 1.0 - alpha
```

`leader_open_position`과 `leader_grasp_position`이 같은 경우:

- node는 error log를 출력하고 publish를 중단한다.
- 하드웨어가 움직이지 않도록 마지막 command도 갱신하지 않는다.

## 9. Tesollo Joint Names

Tesollo right hand joint 순서는 `dg5f_right_controller.yaml` 및 `dg5f_right_grasp_test.py`와 동일하게 고정한다.

```text
rj_dg_1_1
rj_dg_1_2
rj_dg_1_3
rj_dg_1_4
rj_dg_2_1
rj_dg_2_2
rj_dg_2_3
rj_dg_2_4
rj_dg_3_1
rj_dg_3_2
rj_dg_3_3
rj_dg_3_4
rj_dg_4_1
rj_dg_4_2
rj_dg_4_3
rj_dg_4_4
rj_dg_5_1
rj_dg_5_2
rj_dg_5_3
rj_dg_5_4
```

## 10. 초기 자세

초기 자세는 모든 joint를 `0.0 rad`로 둔다.

```text
q_initial = [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]
```

## 11. Grasp 자세

grasp 자세는 `dg5f_right_grasp_test.py`의 `grasp_deg` 값을 사용한다.

단위는 구현 시 radian으로 변환한다.

| joint | deg |
|---|---:|
| `rj_dg_1_1` | 5.2 |
| `rj_dg_1_2` | -100.5 |
| `rj_dg_1_3` | 41.1 |
| `rj_dg_1_4` | 10.3 |
| `rj_dg_2_1` | 5.9 |
| `rj_dg_2_2` | 21.0 |
| `rj_dg_2_3` | 57.0 |
| `rj_dg_2_4` | 34.7 |
| `rj_dg_3_1` | 4.7 |
| `rj_dg_3_2` | 27.7 |
| `rj_dg_3_3` | 60.2 |
| `rj_dg_3_4` | 38.7 |
| `rj_dg_4_1` | 2.2 |
| `rj_dg_4_2` | 34.4 |
| `rj_dg_4_3` | 53.8 |
| `rj_dg_4_4` | 40.4 |
| `rj_dg_5_1` | 3.8 |
| `rj_dg_5_2` | 4.4 |
| `rj_dg_5_3` | 44.4 |
| `rj_dg_5_4` | 45.4 |

변환 함수:

```python
def d2r(deg):
    return deg * math.pi / 180.0
```

## 12. Tesollo Command Message

bridge node는 다음 형태로 command를 publish한다.

```python
msg = MultiDOFCommand()
msg.dof_names = joint_names
msg.values = target_positions
msg.values_dot = [0.0] * len(joint_names)
publisher.publish(msg)
```

`dof_names`, `values`, `values_dot` 배열 길이는 항상 20이어야 한다.

```text
len(dof_names) == len(values) == len(values_dot) == 20
```

## 13. 로깅 토픽 설계

rosbag에는 OpenArm right arm 정보와 Tesollo hand 정보를 함께 기록한다.

### OpenArm

| 토픽 | 메시지 타입 | 내용 |
|---|---|---|
| `/openarm/right/joint_states` | `sensor_msgs/msg/JointState` | right follower arm position, velocity, effort |
| `/openarm/right/leader/gripper_state` | `sensor_msgs/msg/JointState` | right leader gripper input |

### Tesollo

| 토픽 | 메시지 타입 | 내용 |
|---|---|---|
| `/dg5f_right/joint_states` | `sensor_msgs/msg/JointState` | 20 joint position, velocity, effort/current |
| `/dg5f_right/fingertip_1_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | finger 1 fx, fy, fz, tx, ty, tz |
| `/dg5f_right/fingertip_2_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | finger 2 fx, fy, fz, tx, ty, tz |
| `/dg5f_right/fingertip_3_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | finger 3 fx, fy, fz, tx, ty, tz |
| `/dg5f_right/fingertip_4_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | finger 4 fx, fy, fz, tx, ty, tz |
| `/dg5f_right/fingertip_5_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | finger 5 fx, fy, fz, tx, ty, tz |

주의:

- `delto_hardware/src/system_interface.cpp`는 `received_data.current`를 `efforts_`에 넣는다.
- 따라서 `/dg5f_right/joint_states.effort`는 실제 물리 torque라기보다 current 기반 effort 값일 가능성이 높다.
- 정확한 단위는 Tesollo SDK 문서 또는 실측으로 확인해야 한다.

## 14. 기록 명령 예시

right arm과 Tesollo right hand를 함께 기록한다.

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

## 15. 권장 실행 순서

Terminal 1: CAN 설정

```bash
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can3 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan-4-arms -fd
```

Terminal 2: OpenArm right teleoperation

```bash
cd /home/user/Desktop/ros2_ws/src/openarm_teleop
./script/launch_unilateral.sh right_arm can0 can2
```

Terminal 3: Tesollo right hand

```bash
cd /home/user/Desktop/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_driver.launch.py fingertip_sensor:=true
```

Terminal 4: Tesollo bridge

```bash
cd /home/user/Desktop/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=126 ros2 run openarm_teleop tesollo_right_hand_setting_bridge
```

Terminal 5: rosbag record

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

## 16. Build 변경 사항

bridge node를 Python script로 구현하면 `openarm_teleop/CMakeLists.txt`에 install rule을 추가한다.

예시:

```cmake
install(
  PROGRAMS
    script/tesollo_right_hand_setting_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`package.xml`에는 런타임 의존성이 필요하다.

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>control_msgs</depend>
```

## 17. 검증 절차

### 17.1 Build

```bash
cd /home/user/Desktop/ros2_ws
colcon build --packages-select openarm_teleop dg5f_driver delto_hardware
source install/setup.bash
```

### 17.2 Tesollo driver 확인

```bash
ROS_DOMAIN_ID=126 ros2 topic list | grep dg5f_right
ROS_DOMAIN_ID=126 ros2 topic echo /dg5f_right/joint_states --once
ROS_DOMAIN_ID=126 ros2 topic echo /dg5f_right/fingertip_1_broadcaster/wrench --once
```

### 17.3 Leader gripper 입력 확인

```bash
ROS_DOMAIN_ID=126 ros2 topic echo /openarm/right/leader/gripper_state
```

leader right gripper를 움직였을 때 position 값이 연속적으로 변해야 한다.

### 17.4 Bridge 출력 확인

```bash
ROS_DOMAIN_ID=126 ros2 topic echo /dg5f_right/rj_dg_pospid/reference
```

leader gripper를 움직였을 때:

- `values` 배열 길이가 20이어야 한다.
- open 위치에서 모든 값은 0.0 근처여야 한다.
- grasp 위치에서 `grasp_deg`를 radian 변환한 값 근처여야 한다.
- 중간 위치에서는 초기 자세와 grasp 자세 사이 값이어야 한다.

### 17.5 Rosbag 확인

```bash
ros2 bag info <bag_dir>
```

다음 토픽이 모두 포함되어야 한다.

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

## 18. Acceptance Criteria

Plan2 구현 완료 기준:

- OpenArm right teleoperation 실행 중 `/openarm/right/leader/gripper_state`가 publish된다.
- Tesollo right driver 실행 중 `/dg5f_right/joint_states`가 publish된다.
- `fingertip_sensor:=true` 실행 시 5개 fingertip wrench 토픽이 publish된다.
- leader right gripper를 연속적으로 움직이면 Tesollo right hand가 초기 자세에서 grasp 자세까지 연속적으로 따라 움직인다.
- Tesollo command의 `dof_names`, `values`, `values_dot` 길이는 항상 20이다.
- rosbag에 OpenArm right arm, leader gripper input, Tesollo joint state, 5개 fingertip wrench가 모두 기록된다.
- bag replay 시 Tesollo joint position, velocity, effort/current와 fingertip `fx, fy, fz, tx, ty, tz`를 확인할 수 있다.

## 19. 후속 확장

Plan2 이후 확장 후보:

- leader gripper 입력 min/max 자동 calibration node 추가
- Tesollo target posture를 YAML 파일로 분리
- grasp 자세를 object type별 recipe로 관리
- fingertip force feedback 기반 grasp force limiter 추가
- `/dg5f_right/joint_states.effort`의 정확한 단위 검증 및 별도 current topic 분리
- OpenArm + Tesollo + camera timestamp alignment 분석 스크립트 추가
