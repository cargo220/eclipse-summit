#!/usr/bin/env bash
# Robot description stack on Jetson (GPS/IMU/EKF/wheels).
#
# Optional YOLO TRT (separate containers; same ROS_DOMAIN_ID):
#   ENABLE_YOLO_TRT=true ./scripts/run_jetson_description.sh
#
# YOLO uses scripts/run_yolo_trt_live.sh (humble-yolo + humble-yolo-trt).
# Pass-through YOLO env: YOLO_*, REBUILD_ECLIPSE_PKG, DEBUG_MAX_*, H264_*, START_REALSENSE, ...
#
#   YOLO_TRT_STOP_ON_EXIT=true   # default: stop RS+YOLO when description exits
#   YOLO_TRT_STOP_ON_EXIT=false  # leave YOLO stack running after Ctrl-C
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ECLIPSE_WORKSPACE="${ECLIPSE_WORKSPACE:-/home/tars/ewooni_docker_6}"

ENABLE_YOLO_TRT="${ENABLE_YOLO_TRT:-false}"
YOLO_TRT_STOP_ON_EXIT="${YOLO_TRT_STOP_ON_EXIT:-true}"
STARTED_YOLO=false

cleanup() {
  local code=$?
  if [ "$STARTED_YOLO" = true ] && [ "$YOLO_TRT_STOP_ON_EXIT" = true ]; then
    echo "stopping YOLO TRT stack (YOLO_TRT_STOP_ON_EXIT=true)..."
    "$SCRIPT_DIR/run_yolo_trt_live.sh" stop || true
  elif [ "$STARTED_YOLO" = true ]; then
    echo "leaving YOLO TRT stack running (YOLO_TRT_STOP_ON_EXIT=false)"
    echo "  stop later: $SCRIPT_DIR/run_yolo_trt_live.sh stop"
  fi
  exit "$code"
}

export ECLIPSE_LAUNCH_FILE="description.launch.py"
export ECLIPSE_CONTAINER_NAME="${ECLIPSE_CONTAINER_NAME:-eclipse_test3_description_live}"

if [ "$ENABLE_YOLO_TRT" = true ]; then
  echo "ENABLE_YOLO_TRT=true → starting RealSense + YOLO TRT first"
  "$SCRIPT_DIR/run_yolo_trt_live.sh" start
  STARTED_YOLO=true
  trap cleanup EXIT INT TERM
  echo "YOLO TRT up; starting description (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-27})"
else
  echo "ENABLE_YOLO_TRT=false (description only)"
fi

# No exec: keep this shell so trap can stop YOLO after description exits.
"$SCRIPT_DIR/run_jetson_launch.sh" "$@"
