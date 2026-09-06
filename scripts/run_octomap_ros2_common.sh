#!/usr/bin/env bash

# Shared ROS 2 process orchestration for the dataset-specific wrappers.
run_octomap_ros2_pipeline() {
  : "${ROOT_DIR:?ROOT_DIR is required}"
  : "${SLAM_BIN:?SLAM_BIN is required}"
  : "${RVIZ_CONFIG:?RVIZ_CONFIG is required}"
  : "${RUN_LABEL:?RUN_LABEL is required}"

  local start_rviz="${START_RVIZ:-1}"
  local octomap_resolution="${OCTOMAP_RESOLUTION:-0.05}"
  local octomap_max_range="${OCTOMAP_MAX_RANGE:-8.0}"
  local octomap_pid=""
  local rviz_pid=""
  local slam_pid=""
  local shutting_down=0

  set +u
  source "${ROOT_DIR}/scripts/setup_humble.sh"
  set -u

  if [ ! -x "${SLAM_BIN}" ]; then
    echo "[${RUN_LABEL}] Missing executable: ${SLAM_BIN}" >&2
    echo "[${RUN_LABEL}] Run ./build_octomap.sh first." >&2
    return 1
  fi

  topic_has_publisher() {
    local topic="$1"
    local info
    if ! info="$(ros2 topic info "${topic}" 2>/dev/null)"; then
      return 1
    fi
    printf '%s\n' "${info}" | grep -Eq '^Publisher count: [1-9]'
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
    local timeout_s="${2:-45}"
    local deadline=$((SECONDS + timeout_s))
    while [ "${SECONDS}" -lt "${deadline}" ]; do
      if timeout 2 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
        return 0
      fi
      sleep 0.5
    done
    return 1
  }

  cleanup_ros2_pipeline() {
    if [ "${shutting_down}" -ne 0 ]; then
      return
    fi
    shutting_down=1
    if [ -n "${slam_pid}" ]; then
      kill "${slam_pid}" >/dev/null 2>&1 || true
      wait "${slam_pid}" >/dev/null 2>&1 || true
    fi
    if [ -n "${rviz_pid}" ]; then
      kill "${rviz_pid}" >/dev/null 2>&1 || true
    fi
    if [ -n "${octomap_pid}" ]; then
      kill "${octomap_pid}" >/dev/null 2>&1 || true
    fi
  }
  trap cleanup_ros2_pipeline INT TERM EXIT

  ros2 run octomap_server octomap_server_node --ros-args \
    -r cloud_in:=/AYG/Local_Point_Clouds \
    -p frame_id:=AYG/map \
    -p resolution:="${octomap_resolution}" \
    -p sensor_model.max_range:="${octomap_max_range}" \
    -p filter_ground:=false \
    -p filter_speckles:=true >"/tmp/${RUN_LABEL}_octomap.log" 2>&1 &
  octomap_pid=$!
  if ! wait_for_topic_publisher /octomap_binary 20; then
    echo "[${RUN_LABEL}] octomap_server did not advertise /octomap_binary." >&2
    return 1
  fi

  if [ "${start_rviz}" != "0" ]; then
    rviz2 -d "${RVIZ_CONFIG}" >"/tmp/${RUN_LABEL}_rviz.log" 2>&1 &
    rviz_pid=$!
  fi

  echo "[${RUN_LABEL}] Starting SLAM after octomap_server is ready..."
  "${SLAM_BIN}" "${SLAM_ARGS[@]}" &
  slam_pid=$!
  if ! wait_for_topic_publisher /AYG/Local_Point_Clouds 30; then
    echo "[${RUN_LABEL}] SLAM did not advertise /AYG/Local_Point_Clouds." >&2
    return 1
  fi
  if ! wait_for_topic_message /octomap_binary 45; then
    echo "[${RUN_LABEL}] /octomap_binary did not receive a map message." >&2
    return 1
  fi

  wait "${slam_pid}"
}
