#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VOCAB_PATH="${1:-${ROOT_DIR}/Vocabulary/ORBvoc.txt}"
SETTINGS_PATH="${2:-${ROOT_DIR}/Examples/Stereo/EuRoC.yaml}"
LEFT_DIR="${3:-/path/to/dataset}"
RIGHT_DIR="${4:-/path/to/dataset}"
TIMES_PATH="${5:-${ROOT_DIR}/Examples/Stereo/EuRoC_TimeStamps/MH01.txt}"
RVIZ_CONFIG="${ROOT_DIR}/Examples/RGB-D/octomap_view_tum.rviz"
SLAM_BIN="${ROOT_DIR}/Examples/Stereo/stereo_euroc_octomap"
RUN_LABEL="ayg_slam_euroc"
SLAM_ARGS=("${VOCAB_PATH}" "${SETTINGS_PATH}" "${LEFT_DIR}" "${RIGHT_DIR}" "${TIMES_PATH}")

source "${ROOT_DIR}/scripts/run_octomap_ros2_common.sh"
run_octomap_ros2_pipeline
