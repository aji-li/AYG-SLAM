#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VOCAB_PATH="${1:-${ROOT_DIR}/Vocabulary/ORBvoc.txt}"
SETTINGS_PATH="${2:-${ROOT_DIR}/Examples/Stereo/KITTI00-02.yaml}"
DATASET_DIR="${3:-/path/to/dataset}"
RVIZ_CONFIG="${ROOT_DIR}/Examples/RGB-D/octomap_view_airsim.rviz"
SLAM_BIN="${ROOT_DIR}/Examples/Stereo/stereo_kitti_octomap"
RUN_LABEL="ayg_slam_kitti"
SLAM_ARGS=("${VOCAB_PATH}" "${SETTINGS_PATH}" "${DATASET_DIR}")

if [ ! -d "${DATASET_DIR}/image_0" ] || [ ! -d "${DATASET_DIR}/image_1" ] || [ ! -f "${DATASET_DIR}/times.txt" ]; then
  echo "[${RUN_LABEL}] Expected image_0/, image_1/ and times.txt under ${DATASET_DIR}." >&2
  exit 1
fi

source "${ROOT_DIR}/scripts/run_octomap_ros2_common.sh"
run_octomap_ros2_pipeline
