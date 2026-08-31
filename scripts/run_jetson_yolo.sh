#!/usr/bin/env bash
# One-shot Jetson entry: RealSense RGB-D + thin YOLO 2D/3D via description.launch.
# Requires image eclipse-v3:humble-yolo (see scripts/setup_jetson_yolo.sh).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ECLIPSE_LAUNCH_FILE="${ECLIPSE_LAUNCH_FILE:-description.launch.py}"
export ECLIPSE_CONTAINER_NAME="${ECLIPSE_CONTAINER_NAME:-eclipse_v3_yolo_live}"
export ECLIPSE_DOCKER_IMAGE="${ECLIPSE_DOCKER_IMAGE:-eclipse-v3:humble-yolo}"
export ENABLE_REALSENSE="${ENABLE_REALSENSE:-true}"
export ENABLE_FRONT_CAMERA="${ENABLE_FRONT_CAMERA:-false}"
export ENABLE_YOLO="${ENABLE_YOLO:-true}"
export ENABLE_YOLO_3D="${ENABLE_YOLO_3D:-true}"
export YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
export YOLO_MODEL="${YOLO_MODEL:-/workspaces/eclipse-v3/yolov8n.pt}"

exec "$SCRIPT_DIR/run_jetson_launch.sh" "$@"
