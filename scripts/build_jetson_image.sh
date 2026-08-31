#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE="${ECLIPSE_DOCKER_IMAGE:-eclipse-test-2:humble}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found" >&2
  exit 1
fi

cd "$WORKSPACE"
echo "workspace: $WORKSPACE"
echo "image: $IMAGE"

docker build \
  -t "$IMAGE" \
  -f docker/jetson_test2.Dockerfile \
  .
