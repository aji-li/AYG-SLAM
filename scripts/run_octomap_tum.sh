#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VOCAB_PATH="${1:-${ROOT_DIR}/Vocabulary/ORBvoc.txt}"
SETTINGS_PATH="${2:-${ROOT_DIR}/Examples/RGB-D/TUM3.yaml}"
DATASET_DIR="${3:-/path/to/dataset}"
ASSOC_PATH="${4:-/path/to/dataset}"
RVIZ_CONFIG="${ROOT_DIR}/Examples/RGB-D/octomap_view_tum.rviz"
SLAM_BIN="${ROOT_DIR}/Examples/RGB-D/rgbd_tum_octomap"
START_RVIZ="${START_RVIZ:-1}"
OCTOMAP_PID=""
RVIZ_PID=""
SLAM_PID=""
DIAG_PID=""
SHUTTING_DOWN=0

# ROS 2's generated setup files are not compatible with Bash nounset.
set +u
source "${ROOT_DIR}/scripts/setup_humble.sh"
set -u

topic_has_publisher() {
  local topic="$1"
  local info
  if ! info="$(ros2 topic info "${topic}" 2>/dev/null)"; then
    return 1
  fi
  printf '%s\n' "${info}" | grep -Eq "^Publisher count: [1-9]"
}

wait_for_topic_publisher() {
  local topic="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if topic_has_publisher "${topic}"; then
      return 0
    fi
    sleep 0.3
  done
  return 1
}

wait_for_topic_message() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local deadline=$((SECONDS + timeout_s))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if timeout 2 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

cleanup() {
  if [ "${SHUTTING_DOWN}" -ne 0 ]; then
    return
  fi
  SHUTTING_DOWN=1

  if [ -n "${DIAG_PID}" ]; then
    kill "${DIAG_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${SLAM_PID}" ]; then
    kill "${SLAM_PID}" >/dev/null 2>&1 || true
    wait "${SLAM_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${RVIZ_PID}" ]; then
    kill "${RVIZ_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${OCTOMAP_PID}" ]; then
    kill "${OCTOMAP_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup INT TERM EXIT

ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/AYG/Local_Point_Clouds \
  -p frame_id:=AYG/map \
  -p resolution:=0.05 \
  -p sensor_model.max_range:=8.0 \
  -p filter_ground:=false \
  -p filter_speckles:=true >/tmp/ayg_slam_octomap.log 2>&1 &
OCTOMAP_PID=$!
if ! wait_for_topic_publisher /octomap_binary 20; then
  echo "[run_octomap_tum] octomap_server did not advertise /octomap_binary within 20s." >&2
  exit 1
fi

if [ "${START_RVIZ}" != "0" ]; then
  rviz2 -d "${RVIZ_CONFIG}" >/tmp/ayg_slam_rviz.log 2>&1 &
  RVIZ_PID=$!
fi

echo "[run_octomap_tum] Starting SLAM after octomap_server is ready..."
"${SLAM_BIN}" "${VOCAB_PATH}" "${SETTINGS_PATH}" "${DATASET_DIR}" "${ASSOC_PATH}" &
SLAM_PID=$!
if ! wait_for_topic_publisher /AYG/Local_Point_Clouds 30; then
  echo "[run_octomap_tum] SLAM did not advertise /AYG/Local_Point_Clouds within 30s." >&2
  exit 1
fi

if ! wait_for_topic_message /octomap_binary 45; then
  echo "[run_octomap_tum] /octomap_binary did not receive any map message within 45s." >&2
  exit 1
fi

(
  if ros2 topic info /octomap_binary >/tmp/ayg_slam_octomap_topic.info 2>&1; then
    echo "[run_octomap_tum] /octomap_binary info:"
    cat /tmp/ayg_slam_octomap_topic.info
  else
    echo "[run_octomap_tum] /octomap_binary not available yet"
  fi
) &
DIAG_PID=$!
wait "${SLAM_PID}"
