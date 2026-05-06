#!/usr/bin/env bash
set -euo pipefail

RECORD_TIME_SEC="10"
OUTPUT_NAME="output"
STORAGE_ID="sqlite3"
MAX_CACHE_SIZE="100000000"
DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
USE_DEFAULT_TOPICS="1"

DEFAULT_TOPICS=(
    "/openarm/left/joint_states"
    "/openarm/right/joint_states"
    "/openarm/left/leader/gripper_state"
    "/dg5f_right/rj_dg_pospid/reference"
    "/dg5f_right/joint_states"
    "/tesollo/right/sensor"
    "/tf"
    "/tf_static"
)

TOPICS=()
EXTRA_RECORD_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  ./script/record_bag_with_enter.sh [wrapper options] [topics...]
  ./script/record_bag_with_enter.sh [wrapper options] -- [ros2 bag record extra args...]

Example:
  ./script/record_bag_with_enter.sh --record-time 10 --output output
  ./script/record_bag_with_enter.sh --record-time 10 /tf /tf_static
  ./script/record_bag_with_enter.sh --record-time 10 --topic /tf --topic /tf_static
  ./script/record_bag_with_enter.sh --record-time 10 -- --include-hidden-topics

Wrapper options:
  --record-time SEC       Recording time in seconds (default: 10)
  --output NAME           Bag output name/folder (default: output)
  --domain-id ID          ROS_DOMAIN_ID (default: 126 or current env)
  --storage ID            ros2 bag storage id (default: sqlite3)
  --max-cache-size BYTES  ros2 bag cache size (default: 100000000)
  --topic TOPIC           Add topic to record (repeatable)
  --no-default-topics     Do not include built-in default topic list
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --record-time)
            RECORD_TIME_SEC="$2"
            shift 2
            ;;
        --output)
            OUTPUT_NAME="$2"
            shift 2
            ;;
        --domain-id)
            DOMAIN_ID="$2"
            shift 2
            ;;
        --storage)
            STORAGE_ID="$2"
            shift 2
            ;;
        --max-cache-size)
            MAX_CACHE_SIZE="$2"
            shift 2
            ;;
        --topic)
            TOPICS+=("$2")
            shift 2
            ;;
        --no-default-topics)
            USE_DEFAULT_TOPICS="0"
            shift
            ;;
        --)
            shift
            while [[ $# -gt 0 ]]; do
                EXTRA_RECORD_ARGS+=("$1")
                shift
            done
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            # Positional args are treated as topic names for convenience.
            TOPICS+=("$1")
            shift
            ;;
    esac
done

if ! [[ "$RECORD_TIME_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "--record-time must be a positive number (seconds)." >&2
    exit 1
fi

if ! [[ "$MAX_CACHE_SIZE" =~ ^[0-9]+$ ]]; then
    echo "--max-cache-size must be an integer (bytes)." >&2
    exit 1
fi

if [[ "${#TOPICS[@]}" -eq 0 && "${USE_DEFAULT_TOPICS}" == "1" ]]; then
    TOPICS=("${DEFAULT_TOPICS[@]}")
fi

if [[ "${#TOPICS[@]}" -eq 0 ]]; then
    echo "No topics selected. Provide topics or remove --no-default-topics." >&2
    exit 1
fi

echo "[대기] Enter를 누르면 ${RECORD_TIME_SEC}초 기록을 시작합니다."
read -r

CMD=(
    env ROS_DOMAIN_ID="${DOMAIN_ID}"
    ros2 bag record
    -o "${OUTPUT_NAME}"
    -s "${STORAGE_ID}"
    --max-cache-size "${MAX_CACHE_SIZE}"
)
CMD+=("${EXTRA_RECORD_ARGS[@]}")
CMD+=("${TOPICS[@]}")

timeout --signal=INT --kill-after=3s "${RECORD_TIME_SEC}s" "${CMD[@]}"
