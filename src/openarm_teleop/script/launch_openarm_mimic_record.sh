#!/usr/bin/env bash
# launch_openarm_mimic_record.sh — Record OpenArm teleoperation demos via Isaac Lab Mimic
#
# Usage:
#   ./script/launch_openarm_mimic_record.sh
#   ./script/launch_openarm_mimic_record.sh --num_demos 10
#   ./script/launch_openarm_mimic_record.sh --num_demos 10 --output_file ./datasets/openarm_lift_demos.hdf5
#
# Requires:
#   - IsaacSim 5.1.0 + IsaacLab 2.3.1 at /home/usr/rl_ws/IsaacLab
#   - OpenArm + Tesollo hardware connected and ROS2 publishing on domain 126
#   - ROS2 topics: /openarm/right/joint_states, /dg5f_right/rj_dg_pospid/reference
#
# HDF5 output structure (written by record_demos.py):
#   data/demo_N/actions         [T, 7]  delta_pos(3)+delta_rot_aa(3)+gripper(1)
#   data/demo_N/initial_state   sim state snapshot
#   data/demo_N/obs/eef_pos     [T, 3]
#   data/demo_N/obs/eef_quat    [T, 4]
#   data/demo_N/obs/joint_pos   [T, N]
#
# Controls once running:
#   L      start / stop recording an episode
#   Q      quit

set -euo pipefail

ISAACLAB_ROOT="/home/usr/rl_ws/IsaacLab"
RECORD_SCRIPT="$ISAACLAB_ROOT/scripts/tools/record_demos.py"
TASK="Isaac-Lift-Cube-OpenArm-IK-Rel-Mimic-v0"
OUTPUT_FILE="./datasets/openarm_lift_mimic_demos.hdf5"

export ROS_DOMAIN_ID=126

# Source ROS2 workspace so topics are accessible in the same process
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    source /home/usr/ros2_ws/install/setup.bash
fi

exec "$ISAACLAB_ROOT/isaaclab.sh" -p "$RECORD_SCRIPT" \
    --task "$TASK" \
    --teleop_device openarm_ros2 \
    --output_file "$OUTPUT_FILE" \
    --num_envs 1 \
    "$@"
