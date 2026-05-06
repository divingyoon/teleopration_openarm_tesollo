#!/usr/bin/env bash
# Backward-compatible wrapper for the unified pour_v1_mimic pipeline entrypoint.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUT_FILE="./datasets/openarm_lift_mimic_demos.hdf5"
OUTPUT_FILE="./datasets/openarm_lift_mimic_generated.hdf5"
NUM_ENVS=1
GENERATION_NUM_TRIALS=200

while [[ $# -gt 0 ]]; do
    case "$1" in
        --input)    INPUT_FILE="$2"; shift 2 ;;
        --output)   OUTPUT_FILE="$2"; shift 2 ;;
        --num_envs) NUM_ENVS="$2"; shift 2 ;;
        --generation_num_trials) GENERATION_NUM_TRIALS="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

exec python3 "${SCRIPT_DIR}/run_pour_v1_mimic_pipeline.py" \
    annotate_generate \
    --input-file "${INPUT_FILE}" \
    --output-file "${OUTPUT_FILE}" \
    --num-envs "${NUM_ENVS}" \
    --generation-num-trials "${GENERATION_NUM_TRIALS}"
