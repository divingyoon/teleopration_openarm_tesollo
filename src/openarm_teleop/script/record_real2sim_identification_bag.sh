#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-./bags/real2sim_identification}"
RECORD_TIME_SEC="${2:-20}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"

mkdir -p "$OUTPUT_DIR"

TOPICS=(
  # bilateral teleop 스택의 팔 상태 (명령 입력이 없는 경로)
  /openarm/left/joint_states
  /openarm/right/joint_states
  /openarm/left/leader/gripper_state
  /openarm/right/leader/gripper_state
  # 손: 명령(MultiDOFCommand, dof_names 포함) + 측정
  /dg5f_right/rj_dg_pospid/reference
  /dg5f_right/joint_states
  # 팔: forward_position_controller 로 띄웠을 때의 명령 + ros2_control 상태
  # Float64MultiArray 에는 이름이 없다. 변환 시 컨트롤러 yaml의 joints 순서를 넘겨야 한다.
  /right_forward_position_controller/commands
  /joint_states
  /tesollo/right/joint_states
  /tesollo/right/sensor
  /tf
  /tf_static
)

echo "[INFO] Recording Real2Sim identification bag"
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] output=$OUTPUT_DIR"
echo "[INFO] duration=${RECORD_TIME_SEC}s"

export ROS_DOMAIN_ID
timeout "${RECORD_TIME_SEC}s" ros2 bag record \
  -o "$OUTPUT_DIR" \
  -s sqlite3 \
  --max-cache-size 100000000 \
  "${TOPICS[@]}"
