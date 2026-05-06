# OpenArm + Tesollo Right Hand Teleop Guide

## Overview
- Goal: link `master gripper` motion to `Tesollo right hand`.
- Required terminals: `Terminal 1` (OpenArm), `Terminal 2` (Tesollo + bridge), `Terminal 3` (bag recording).
- ROS domain: `126`.

## Pour V1 Mimic Pipeline (DB3 canonical)
- Unified entrypoint: `src/openarm_teleop/script/run_pour_v1_mimic_pipeline.py`
- Scope: **pre-pour only** (`grasp -> lift -> align`, no full pour BC in this stage)
- Gym task IDs:
  - Mimic generation: `Pour-Mimic-V1-Mimic-v0`
  - Eval/PPO: `Pour-Mimic-V1-v0`
- Action contract: `18D`
  - `[0:6]` right palm delta
  - `[6:11]` right hand curl (5D from 20D hand reference)
  - `[11:18]` left arm delta
- Canonical source data is `rosbag2 DB3`; HDF5 is derived training data.
- Pre-pour metadata intent is fixed to `pre_pour_init`.
- Default collection topics are unchanged; if needed, extend with additional raw fingertip wrench topics via `collect_db3 --topics ...`.

Examples:
```bash
# 1) Collect DB3 with session metadata
python3 src/openarm_teleop/script/run_pour_v1_mimic_pipeline.py collect_db3 \
  --output-dir /home/usr/ros2_ws/bags --record-time-sec 12 --operator op_a --attempt-id a01

# 2) Convert DB3 -> HDF5 (100Hz sync, contract validation)
python3 src/openarm_teleop/script/run_pour_v1_mimic_pipeline.py convert_hdf5 \
  --bag-dirs /home/usr/ros2_ws/bags/pour_v1_20260101T120000Z \
  --output-file /home/usr/ros2_ws/datasets/pour_v1_mimic_raw.hdf5

# 2-b) Phase 4 / Method B (fixed cup pose, static)
python3 src/openarm_teleop/script/run_pour_v1_mimic_pipeline.py convert_hdf5 \
  --bag-dirs /home/usr/ros2_ws/bags/pour_v1_20260101T120000Z \
  --output-file /home/usr/ros2_ws/datasets/pour_v1_mimic_raw.hdf5 \
  --object-pose-mode static \
  --source-cup-pose-w "1,0,0,0.35,0,1,0,-0.10,0,0,1,0.05,0,0,0,1" \
  --target-cup-pose-w "1,0,0,0.35,0,1,0,0.10,0,0,1,0.05,0,0,0,1"

# 2-c) Phase 4 / Method A (TF object pose + static fallback)
# converter reads cup pose from /tf,/tf_static chain; if unavailable at a step, static pose is used.
#   --object-pose-mode tf --tf-topic /tf \
#   --source-cup-frame source_cup --target-cup-frame target_cup --tf-reference-frame world

# 3) Mimic annotation + generation
python3 src/openarm_teleop/script/run_pour_v1_mimic_pipeline.py annotate_generate \
  --input-file /home/usr/ros2_ws/datasets/pour_v1_mimic_raw.hdf5 \
  --output-file /home/usr/ros2_ws/datasets/pour_v1_mimic_aug.hdf5 \
  --generation-num-trials 200

# 4) BC pre-pour train (truncate each episode at align_done)
python3 src/openarm_teleop/script/run_pour_v1_mimic_pipeline.py train_bc \
  --dataset /home/usr/ros2_ws/datasets/pour_v1_mimic_aug.hdf5 \
  --truncate-at align_done \
  --run-name pour_v1_pre_pour_bc
```

## Terminal 1 (OpenArm, `~/ros2_ws/src/openarm_teleop`)
```bash
# usb 연결 순서 : leader -> follower
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can3 -fd -b 1000000 -d 5000000

openarm-can-configure-socketcan-4-arms -fd

cd /home/usr/ros2_ws/src/openarm_teleop
```

### 1-A) Unilateral (단방향 제어)
```bash
./script/launch_unilateral.sh left_arm can3 can1
./script/launch_unilateral.sh right_arm can2 can0
```

### 1-B) Bilateral (양방향 힘 반영 제어)
```bash
./script/launch_bilateral.sh left_arm can3 can1
./script/launch_bilateral.sh right_arm can2 can0
```

## Terminal 2 (Tesollo Hand + Bridge, `~/ros2_ws`)
```bash
cd /home/usr/ros2_ws
source install/setup.bash
```

### 2-1) Tesollo NIC 설정 (최초/재부팅 후 필요)
```bash
# Tesollo 연결된 유선 NIC: enp0s31f6
sudo ip addr flush dev enp0s31f6
sudo ip addr add 169.254.186.10/16 dev enp0s31f6
sudo ip link set enp0s31f6 up

ping -c 3 169.254.186.72
```

### 2-2) Tesollo controller 실행
```bash
ROS_DOMAIN_ID=126 ros2 launch dg5f_driver dg5f_right_pid_all_controller.launch.py \
  delto_ip:=169.254.186.72 \
  fingertip_sensor:=true
```

### 2-3) Master Gripper -> Tesollo Right Hand bridge 실행
```bash
# 별도 터미널에서 실행 권장 (Terminal 2-2는 유지)
cd /home/usr/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=126 ros2 run openarm_teleop tesollo_right_hand_teleop_bridge.py
```
Bridge startup behavior:
- On startup, bridge automatically calls F/T zero service once:
  - `/dg5f_right/delto_hardware_interface_node/set_ft_sensor_offset`
- It waits up to `15s` by default. If the service is not ready, bridge keeps running and logs a warning.

### 2-4) 연결 상태 확인
```bash
ROS_DOMAIN_ID=126 ros2 topic info /openarm/right/leader/gripper_state
ROS_DOMAIN_ID=126 ros2 topic info /dg5f_right/joint_states
ROS_DOMAIN_ID=126 ros2 topic info /dg5f_right/rj_dg_pospid/reference
```

Expected:
- `/openarm/right/leader/gripper_state`: `Publisher count > 0`
- `/dg5f_right/joint_states`: `Publisher count > 0`
- `/dg5f_right/rj_dg_pospid/reference`: `Publisher count > 0`, `Subscription count > 0`

### 2-5) F/T 수동 재보정 (필요 시)
```bash
# 손가락 무접촉/정지 상태에서 실행
ROS_DOMAIN_ID=126 ros2 service call \
  /dg5f_right/delto_hardware_interface_node/set_ft_sensor_offset \
  std_srvs/srv/Trigger {}
```

Optional bridge args for F/T auto calibration:
```bash
# 자동 보정 비활성화
ROS_DOMAIN_ID=126 ros2 run openarm_teleop tesollo_right_hand_teleop_bridge.py \
  --ros-args -p auto_calibrate_ft_sensor:=false

# 서비스명/대기시간 변경
ROS_DOMAIN_ID=126 ros2 run openarm_teleop tesollo_right_hand_teleop_bridge.py \
  --ros-args \
  -p ft_offset_service:=/dg5f_right/delto_hardware_interface_node/set_ft_sensor_offset \
  -p ft_offset_wait_timeout_sec:=20.0
```

# Terminal 3 (root@usr:/workspaces/isaac_ros-dev) 

xhost +SI:localuser:root

# docker ps로 봤을 때 a126_foundationpose container 안켜져 있으면 start로 켜기 
docker start a126_foundationpose
docker exec -it a126_foundationpose /bin/bash

source ${ISAAC_ROS_WS}/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=126

ros2 launch isaac_ros_foundationpose isaac_ros_foundationpose_realsense_yolov8.launch.py   yolov8_model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx   yolov8_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan   refine_model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_model.onnx   refine_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan   score_model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_model.onnx   score_engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan   mesh_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Cup/Cup.obj   texture_path:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Cup/materials/textures/baked_cup.png   num_classes:=1


## Terminal 4 (rosbag, `~/ros2_ws`)
```bash
cd /home/usr/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=126 ros2 bag record -o output -s sqlite3 --max-cache-size 100000000 \
  /openarm/left/joint_states \
  /openarm/right/joint_states \
  /dg5f_right/rj_dg_pospid/reference \
  /dg5f_right/joint_states \
  /tesollo/right/sensor \
  /tf \
  /tf_static
```

Alternative (Enter 누른 시점부터 N초만 자동 기록):
```bash
cd /home/usr/ros2_ws/src/openarm_teleop
export ROS_DOMAIN_ID=126
./script/record_bag_with_enter.sh --record-time 10 --output output \
  /openarm/left/joint_states \
  /openarm/right/joint_states \
  /dg5f_right/rj_dg_pospid/reference \
  /dg5f_right/joint_states \
  /tesollo/right/sensor \
  /tf \
  /tf_static
```
- 실행 후 `[대기]` 메시지에서 `Enter`를 누르면 그 시점부터 `10초` 기록 후 자동 종료됩니다.

Quick check after stop (`Ctrl+C`):
```bash
ros2 bag info output
```

## Notes
- `tesollo_right_hand_teleop_bridge.py` default calibration is already set for this setup:
  - `leader_open_position=0.010490577553978753`
  - `leader_grasp_position=-1.1041809719996944`
  - `invert_input=true`
 
## 테솔로 그랩 자세 관련.
  - Tesollo 디폴트 자세(포즈 20개 배열)
    src/openarm_teleop/script/tesollo_bridge_logic.py:18 의 HAND_APPROACH_POSE
    src/openarm_teleop/script/tesollo_bridge_logic.py:26 의 HAND_GRASP_POSE
  - Readme에 적힌 캘리브레이션 값 (leader_open_position, leader_grasp_position, invert_input)
    src/openarm_teleop/script/tesollo_right_hand_teleop_bridge.py:60~src/openarm_teleop/script/
    tesollo_right_hand_teleop_bridge.py:62

  참고로 런타임에서 파일 수정 없이 포즈도 덮어쓸 수 있습니다 (pose1_rad, pose2_rad 파라미터).
 

- `tesollo_right_hand_teleop_bridge.py` now runs F/T sensor zero automatically at startup by default:
  - `auto_calibrate_ft_sensor=true`
  - `ft_offset_service=/dg5f_right/delto_hardware_interface_node/set_ft_sensor_offset`
  - `ft_offset_wait_timeout_sec=15.0`
  

- If hand direction is wrong, run bridge with:
```bash
ROS_DOMAIN_ID=126 ros2 run openarm_teleop tesollo_right_hand_teleop_bridge.py --ros-args -p invert_input:=false
```
- Topic selection policy used above:
  - DG5F joints use raw source topic: `/dg5f_right/joint_states`
  - Tesollo sensor uses relayed analysis topic: `/tesollo/right/sensor`
