# OpenArm Teleoperation 기록 확장 설계도

## 1. 목적

현재 teleoperation 기록은 `/openarm/joint_states` 토픽을 통해 follower OpenArm의 관절 위치 중심으로 저장된다. 앞으로는 left arm과 right arm 각각에 대해 관절 위치뿐 아니라 관절 속도, 토크 등 OpenArm에서 얻을 수 있는 상태 정보를 rosbag에 기록할 수 있도록 확장한다.

이 문서는 실제 C++ 구현을 위한 설계 기준을 정리한다.

## 2. 현재 기록 구조

현재 실행 흐름은 다음과 같다.

```bash
./script/launch_unilateral.sh left_arm can1 can3
```

위 명령에서 인자 의미는 다음과 같다.

| 인자 | 의미 |
|---|---|
| `left_arm` | 제어 대상 arm side |
| `can1` | leader CAN interface |
| `can3` | follower CAN interface |

현재 rosbag 기록 명령은 다음 토픽을 저장한다.

```bash
ROS_DOMAIN_ID=126 ros2 bag record -s sqlite3 --max-cache-size 100000000 \
  /output \
  /openarm/joint_states \
  /color/image_raw/compressed \
  /color/camera_info \
  /aligned_depth_to_color/image_raw \
  /tf
```

현재 `/openarm/joint_states`는 `src/openarm_teleop/control/openarm_unilateral_control.cpp`의 `AdminThread`에서 publish된다.

현재 publish되는 값은 다음과 같다.

| 항목 | 현재 상태 |
|---|---|
| 대상 | follower OpenArm |
| arm side | launch 인자로 선택된 `left_arm` 또는 `right_arm` |
| arm position | publish됨 |
| hand/gripper position | publish됨 |
| velocity | `JointState` 내부에는 있으나 publish 배열에 넣지 않음 |
| effort/torque | `JointState` 내부에는 있으나 현재 대부분 `0.0`으로 전달됨 |
| leader 상태 | publish하지 않음 |

즉 현재 bag에는 OpenArm follower의 position 중심 데이터가 들어가며, leader 입력값은 별도 토픽으로 저장되지 않는다.

## 3. 목표 요구사항

### 3.1 Left arm 기록

left arm follower에서 얻을 수 있는 관절 상태를 기록한다.

기본 기록 대상:

- arm joint position
- arm joint velocity
- arm joint torque
- hand/gripper joint position
- hand/gripper joint velocity
- hand/gripper torque

추가로 CAN motor 객체에서 확인 가능한 값은 후속 확장 대상으로 둔다.

- motor send CAN ID
- motor receive CAN ID
- motor type
- motor enable 상태
- MOS temperature
- rotor temperature

단, 위 추가 값은 `sensor_msgs/msg/JointState`에 직접 넣기 어렵기 때문에 별도 diagnostic/custom message 설계가 필요하다.

### 3.2 Right arm 기록

right arm도 left arm과 동일한 구조로 기록한다.

right arm 기록은 별도의 토픽을 사용해 left arm 데이터와 섞이지 않게 한다.

## 4. 토픽 설계

left/right arm 상태는 분리된 토픽으로 publish한다.

| arm side | 토픽 | 메시지 타입 |
|---|---|---|
| left arm | `/openarm/left/joint_states` | `sensor_msgs/msg/JointState` |
| right arm | `/openarm/right/joint_states` | `sensor_msgs/msg/JointState` |

기존 `/openarm/joint_states`는 신규 설계의 필수 토픽으로 사용하지 않는다. 기존 분석 코드와 호환이 필요하면 일정 기간 병행 publish하는 방식을 별도 검토한다.

## 5. 메시지 필드 매핑

`sensor_msgs/msg/JointState` 필드는 다음처럼 사용한다.

| JointState 필드 | 기록 값 |
|---|---|
| `header.stamp` | publish 시점의 ROS time |
| `name` | joint 이름 |
| `position` | joint position |
| `velocity` | joint velocity |
| `effort` | motor torque 또는 joint effort |

joint 이름 규칙은 다음과 같이 고정한다.

### Left arm

```text
left_follower_arm_joint_0
left_follower_arm_joint_1
...
left_follower_arm_joint_6
left_follower_hand_joint_0
```

### Right arm

```text
right_follower_arm_joint_0
right_follower_arm_joint_1
...
right_follower_arm_joint_6
right_follower_hand_joint_0
```

배열 길이는 항상 같아야 한다.

```text
len(name) == len(position) == len(velocity) == len(effort)
```

## 6. 구현 변경 지점

### 6.1 `control.cpp`

파일:

```text
src/openarm_teleop/src/controller/control.cpp
```

현재 motor 상태를 읽을 때 torque가 버려진다.

현재 구조 예시:

```cpp
arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
```

변경 방향:

```cpp
arm_motor_states.push_back({
    motor.get_position(),
    motor.get_velocity(),
    motor.get_torque()
});

gripper_motor_states.push_back({
    motor.get_position(),
    motor.get_velocity(),
    motor.get_torque()
});
```

이 변경으로 CAN 응답에서 갱신된 motor torque가 `MotorState.effort`로 들어가고, converter를 통해 `JointState.effort`로 전달된다.

적용 대상 함수:

- `Control::bilateral_step()`
- `Control::unilateral_step()`

현재 unilateral teleop 기록에 직접 필요한 함수는 `Control::unilateral_step()`이지만, 양쪽 제어 모드의 상태 의미를 일관되게 유지하기 위해 두 함수 모두 같은 방식으로 수정하는 것이 좋다.

### 6.2 `openarm_unilateral_control.cpp`

파일:

```text
src/openarm_teleop/control/openarm_unilateral_control.cpp
```

현재 `AdminThread`는 `/openarm/joint_states` 하나만 publish한다.

변경 방향:

1. `arm_side`를 `AdminThread` 생성자에 전달한다.
2. `arm_side == "left_arm"`이면 `/openarm/left/joint_states` publisher를 생성한다.
3. `arm_side == "right_arm"`이면 `/openarm/right/joint_states` publisher를 생성한다.
4. follower response를 publish할 때 `position`, `velocity`, `effort`를 모두 채운다.

publish 로직의 목표 형태:

```cpp
for (size_t i = 0; i < follower_arm_resp.size(); ++i) {
    msg.name.push_back(prefix + "_follower_arm_joint_" + std::to_string(i));
    msg.position.push_back(follower_arm_resp[i].position);
    msg.velocity.push_back(follower_arm_resp[i].velocity);
    msg.effort.push_back(follower_arm_resp[i].effort);
}

for (size_t i = 0; i < follower_hand_resp.size(); ++i) {
    msg.name.push_back(prefix + "_follower_hand_joint_" + std::to_string(i));
    msg.position.push_back(follower_hand_resp[i].position);
    msg.velocity.push_back(follower_hand_resp[i].velocity);
    msg.effort.push_back(follower_hand_resp[i].effort);
}
```

여기서 `prefix`는 다음과 같다.

| `arm_side` | `prefix` |
|---|---|
| `left_arm` | `left` |
| `right_arm` | `right` |

### 6.3 `joint_state_converter.hpp`

파일:

```text
src/openarm_teleop/src/joint_state_converter.hpp
```

현재 converter는 이미 `position`, `velocity`, `effort`를 그대로 전달한다.

```cpp
j[i] = {m[i].position, m[i].velocity, m[i].effort};
```

따라서 converter는 원칙적으로 수정하지 않아도 된다. 다만 추후 motor temperature, CAN ID, enable 상태까지 기록하려면 `JointState`만으로는 부족하므로 별도 상태 구조체 또는 custom ROS message를 설계해야 한다.

## 7. 기록 명령 예시

### 7.1 Left arm 기록

```bash
ROS_DOMAIN_ID=126 ros2 bag record -s sqlite3 --max-cache-size 100000000 \
  /output \
  /openarm/left/joint_states \
  /color/image_raw/compressed \
  /color/camera_info \
  /aligned_depth_to_color/image_raw \
  /tf
```

### 7.2 Right arm 기록

```bash
ROS_DOMAIN_ID=126 ros2 bag record -s sqlite3 --max-cache-size 100000000 \
  /output \
  /openarm/right/joint_states \
  /color/image_raw/compressed \
  /color/camera_info \
  /aligned_depth_to_color/image_raw \
  /tf
```

### 7.3 Left/right 동시 기록

left와 right 프로세스를 각각 실행하는 경우 두 토픽을 함께 기록한다.

```bash
ROS_DOMAIN_ID=126 ros2 bag record -s sqlite3 --max-cache-size 100000000 \
  /output \
  /openarm/left/joint_states \
  /openarm/right/joint_states \
  /color/image_raw/compressed \
  /color/camera_info \
  /aligned_depth_to_color/image_raw \
  /tf
```

## 8. 검증 절차

### 8.1 빌드 확인

```bash
cd /home/user/Desktop/ros2_ws
colcon build --packages-select openarm_teleop
```

### 8.2 토픽 존재 확인

left arm 실행 시:

```bash
ROS_DOMAIN_ID=126 ros2 topic list | grep /openarm
```

기대 결과:

```text
/openarm/left/joint_states
```

right arm 실행 시 기대 결과:

```text
/openarm/right/joint_states
```

### 8.3 메시지 필드 확인

left arm:

```bash
ROS_DOMAIN_ID=126 ros2 topic echo /openarm/left/joint_states --once
```

right arm:

```bash
ROS_DOMAIN_ID=126 ros2 topic echo /openarm/right/joint_states --once
```

확인할 항목:

- `name` 배열이 비어 있지 않아야 한다.
- `position` 배열 길이가 `name`과 같아야 한다.
- `velocity` 배열 길이가 `name`과 같아야 한다.
- `effort` 배열 길이가 `name`과 같아야 한다.
- `effort`가 항상 `0.0`으로만 나오지 않아야 한다.

### 8.4 rosbag 확인

기록 후 다음 명령으로 bag에 토픽이 들어갔는지 확인한다.

```bash
ros2 bag info <bag_directory>
```

확인할 토픽:

```text
/openarm/left/joint_states
/openarm/right/joint_states
```

실행한 arm side에 맞는 토픽이 포함되어야 한다.

## 9. Acceptance Criteria

구현 완료 기준은 다음과 같다.

- left arm 실행 시 `/openarm/left/joint_states`가 publish된다.
- right arm 실행 시 `/openarm/right/joint_states`가 publish된다.
- 각 메시지의 `name`, `position`, `velocity`, `effort` 배열 길이가 동일하다.
- `position`에는 follower 관절 위치가 들어간다.
- `velocity`에는 follower 관절 속도가 들어간다.
- `effort`에는 follower motor torque가 들어간다.
- rosbag record 명령으로 left/right 토픽을 저장할 수 있다.
- 기존 카메라, depth, `/tf`, `/output` 기록 흐름은 변경하지 않는다.

## 10. 후속 확장

`sensor_msgs/msg/JointState`는 position, velocity, effort 기록에는 적합하지만 motor temperature, CAN ID, enable 상태, motor type 같은 메타 정보에는 적합하지 않다.

후속으로 모든 CAN motor 상태를 기록해야 한다면 다음 중 하나를 선택한다.

1. `diagnostic_msgs/msg/DiagnosticArray` 사용
2. `openarm_teleop` 전용 custom message 생성
3. CSV 또는 별도 logging node로 보조 기록

권장 방향은 custom message 생성이다. 다만 현재 요구사항인 관절속도 및 토크 기록은 `sensor_msgs/msg/JointState` 확장만으로 충분하다.
