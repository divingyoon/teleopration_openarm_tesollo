#!/usr/bin/env bash
# launch_record_hdf5.sh — Start HDF5 teleoperation recorder
#
# Usage:
#   ./script/launch_record_hdf5.sh
#   ./script/launch_record_hdf5.sh --output datasets/session_01.hdf5 --hz 10
#
# All arguments are forwarded to record_to_hdf5.py.
# Keyboard controls once running:
#   Space   start / stop episode
#   q / Q   quit and save

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID=126

# Source ROS2 workspace if not already sourced
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    source /home/usr/ros2_ws/install/setup.bash
fi

exec python3 "$SCRIPT_DIR/record_to_hdf5.py" "$@"
