#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${ECLIPSE_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
IMAGE="${ECLIPSE_DOCKER_IMAGE:-eclipse-test-2:humble}"
DOMAIN_ID="${ROS_DOMAIN_ID:-27}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found" >&2
  exit 1
fi

if [ ! -d "$WORKSPACE" ]; then
  echo "workspace not found: $WORKSPACE" >&2
  exit 1
fi

echo "workspace: $WORKSPACE"
echo "image: $IMAGE"
echo "ROS_DOMAIN_ID: $DOMAIN_ID"

exec docker run --rm \
  --network host --ipc host --privileged \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -v /dev:/dev \
  -v "$WORKSPACE":/workspaces/eclipse-test-2 \
  "$IMAGE" \
  bash -lc '
    cd /workspaces/eclipse-test-2
    colcon build --symlink-install \
      --packages-select eclipse_pkg_msgs eclipse_pkg tars_recovery_behaviors tars_tide_layer \
      --build-base build_trt --install-base install_trt
  '
