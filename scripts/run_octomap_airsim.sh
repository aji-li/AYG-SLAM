#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VOCAB_PATH="${1:-${ROOT_DIR}/Vocabulary/ORBvoc.txt}"
SETTINGS_PATH="${2:-${ROOT_DIR}/Examples/RGB-D/airsim_new.yaml}"
DATASET_DIR="${3:-/path/to/dataset}"
ASSOC_PATH="${4:-${DATASET_DIR}/associations.txt}"
RVIZ_CONFIG="${ROOT_DIR}/Examples/RGB-D/octomap_view_airsim.rviz"
SLAM_BIN="${ROOT_DIR}/Examples/RGB-D/rgbd_tum_octomap"
RUN_LABEL="ayg_slam_airsim"
OCTOMAP_RESOLUTION="${OCTOMAP_RESOLUTION:-1.0}"
OCTOMAP_MAX_RANGE="${OCTOMAP_MAX_RANGE:-100.0}"
SLAM_ARGS=("${VOCAB_PATH}" "${SETTINGS_PATH}" "${DATASET_DIR}" "${ASSOC_PATH}")

source "${ROOT_DIR}/scripts/run_octomap_ros2_common.sh"
run_octomap_ros2_pipeline
