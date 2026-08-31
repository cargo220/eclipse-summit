#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tars_host_ros_logs.sh
source "$SCRIPT_DIR/tars_host_ros_logs.sh"
tars_prepare_host_ros_logs

WORKSPACE="${ECLIPSE_WORKSPACE:-/home/tars/ewooni_docker_6}"
IMAGE="${ECLIPSE_DOCKER_IMAGE:-eclipse-test-2:humble}"
CONTAINER="${ECLIPSE_CONTAINER_NAME:-eclipse_test3_launch_live}"
DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
ENABLE_FRONT_CAMERA="${ENABLE_FRONT_CAMERA:-false}"
FRONT_CAMERA_DEVICE="${FRONT_CAMERA_DEVICE:-/dev/video4}"
LAUNCH_FILE="${ECLIPSE_LAUNCH_FILE:-description.launch.py}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found" >&2
  exit 1
fi

if [ ! -d "$WORKSPACE" ]; then
  echo "workspace not found: $WORKSPACE" >&2
  exit 1
fi

if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container is already running: $CONTAINER" >&2
  exit 1
fi

cd "$WORKSPACE"

echo "workspace: $WORKSPACE"
echo "image: $IMAGE"
echo "container: $CONTAINER"
echo "ROS_DOMAIN_ID: $DOMAIN_ID"
echo "enable_front_camera: $ENABLE_FRONT_CAMERA"
echo "front_camera_device: $FRONT_CAMERA_DEVICE"
echo "launch_file: $LAUNCH_FILE"
echo "ROS_LOG_DIR host: $TARS_HOST_LOG_DIR"

# Do not exec: callers (e.g. run_jetson_description.sh) may need EXIT traps.
docker run --rm --name "$CONTAINER" \
  --network host --ipc host --privileged \
  "${TARS_DOCKER_ROS_LOG_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -e ENABLE_FRONT_CAMERA="$ENABLE_FRONT_CAMERA" \
  -e FRONT_CAMERA_DEVICE="$FRONT_CAMERA_DEVICE" \
  -e ECLIPSE_LAUNCH_FILE="$LAUNCH_FILE" \
  -v /dev:/dev \
  -v "$WORKSPACE":/workspaces/eclipse-test-2 \
  "$IMAGE" \
  bash -lc '
    cd /workspaces/eclipse-test-2

    if [ ! -f install_trt/setup.bash ]; then
      echo "workspace install_trt/setup.bash not found; build the TRT install space first (colcon build --install-base install_trt)" >&2
      exit 1
    fi
    source install_trt/setup.bash

    if ! ros2 pkg prefix robot_localization >/dev/null 2>&1; then
      echo "robot_localization package not found in this Docker image." >&2
      echo "Use the eclipse-v2:humble-based image, then retry." >&2
      exit 1
    fi

    if [ "$ENABLE_FRONT_CAMERA" = true ] && ! ros2 pkg prefix usb_cam >/dev/null 2>&1; then
      echo "usb_cam package not found while ENABLE_FRONT_CAMERA=true." >&2
      echo "Install ros-humble-usb-cam in the Docker image or run with ENABLE_FRONT_CAMERA=false." >&2
      exit 1
    fi

    exec ros2 launch eclipse_pkg "$ECLIPSE_LAUNCH_FILE" \
      enable_front_camera:="$ENABLE_FRONT_CAMERA" \
      front_camera_device:="$FRONT_CAMERA_DEVICE" \
      "$@"
  ' launch "$@"
