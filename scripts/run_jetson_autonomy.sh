#!/usr/bin/env bash
# TARS full autonomy stack (single entry point):
#   description (GPS/IMU/EKF/motors) + RealSense + Nav2 + GPS commander + Foxglove
#
# Usage:
#   ./scripts/run_jetson_autonomy.sh                        # named jebu (default)
#   ./scripts/run_jetson_autonomy.sh jebu                   # same as default
#   ./scripts/run_jetson_autonomy.sh start jebu
#   ENABLE_FOXGLOVE=false ./scripts/run_jetson_autonomy.sh
#   ./scripts/run_jetson_autonomy.sh stop
#   ./scripts/run_jetson_autonomy.sh status
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tars_host_ros_logs.sh
source "$SCRIPT_DIR/tars_host_ros_logs.sh"
tars_prepare_host_ros_logs
WORKSPACE="${ECLIPSE_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
IMAGE="${ECLIPSE_DOCKER_IMAGE:-eclipse-test-2:humble}"
CONTAINER="${ECLIPSE_CONTAINER_NAME:-eclipse_test3_autonomy_live}"
DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
ENABLE_FOXGLOVE="${ENABLE_FOXGLOVE:-true}"
# tide_watch GPS source. Default is the real receiver. Never point this
# at a fake publisher on /gps/fix — that contaminates EKF/navsat.
TIDE_GPS_TOPIC="${TIDE_GPS_TOPIC:-/gps/fix}"
# Named waterline. Default jebu. config/keepout_<이름>_perimeter.geojson
# is required for named sites (run-script existence gate).
KEEPOUT_SITE="${KEEPOUT_SITE:-jebu}"
# If set (>= 0), tide_watch draws that baked alpha instead of forecast.
WATERLINE_ALPHA_OVERRIDE="${WATERLINE_ALPHA_OVERRIDE:-}"

# 조석 API 키: 워크스페이스가 아니라 호스트 env 파일에서만 읽는다.
TIDE_ENV_FILE="${TIDE_ENV_FILE:-$HOME/.config/tars/data_go_kr.env}"
if [ -f "$TIDE_ENV_FILE" ]; then
  set -a
  # shellcheck source=/dev/null
  source "$TIDE_ENV_FILE"
  set +a
fi

CMD="${1:-start}"
if [ "$CMD" = "start" ] && [ -n "${2:-}" ]; then
  KEEPOUT_SITE="$2"
elif [ "$CMD" != "start" ] && [ "$CMD" != "stop" ] && [ "$CMD" != "status" ]; then
  KEEPOUT_SITE="$CMD"
  CMD="start"
fi
if [[ "$KEEPOUT_SITE" == */* ]] || [ "$KEEPOUT_SITE" = "." ] || [ "$KEEPOUT_SITE" = ".." ]; then
  echo "KEEPOUT_SITE 는 맵 이름만 넣는다. 경로 금지: $KEEPOUT_SITE" >&2
  exit 2
fi

ROSBRIDGE_CONTAINER="${ECLIPSE_ROSBRIDGE_CONTAINER:-eclipse_test3_rosbridge}"

stop_core() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  echo "stopped: $CONTAINER"
}

stop_all() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  echo "stopped autonomy nodes: $CONTAINER (ROSBridge WebSocket preserved)"
}

status_all() {
  echo "ROS_DOMAIN_ID=$DOMAIN_ID"
  docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' \
    | { head -n 1; grep -E "${CONTAINER}|${ROSBRIDGE_CONTAINER}" || true; }
}

case "$CMD" in
  stop)   stop_all;   exit 0 ;;
  status) status_all; exit 0 ;;
  start) ;;
  *) echo "usage: $0 [start|stop|status]" >&2; exit 2 ;;
esac

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found" >&2; exit 1
fi
if [ ! -d "$WORKSPACE" ]; then
  echo "workspace not found: $WORKSPACE" >&2; exit 1
fi
if docker ps --format "{{.Names}}" | grep -qx "$CONTAINER"; then
  echo "container $CONTAINER is currently running -> stopping old instance before fresh start..."
  stop_core >/dev/null 2>&1 || true
fi

cd "$WORKSPACE"

echo "=== TARS Autonomy Stack ==="
echo "workspace:       $WORKSPACE"
echo "image:           $IMAGE"
echo "container:       $CONTAINER"
echo "ROS_DOMAIN_ID:   $DOMAIN_ID"
echo "enable_foxglove: $ENABLE_FOXGLOVE"
_site_lc="$(printf '%s' "$KEEPOUT_SITE" | tr '[:upper:]' '[:lower:]')"
if [ "$_site_lc" = "tiles" ] || [ "$_site_lc" = "gps" ] || [ "$KEEPOUT_SITE" = "_" ]; then
  echo "keepout_site:    <gps tiles>"
else
  echo "keepout_site:    $KEEPOUT_SITE"
fi
echo "ROS_LOG_DIR host: $TARS_HOST_LOG_DIR"
if [ -n "$KEEPOUT_SITE" ] && [ "$_site_lc" != "tiles" ] && [ "$_site_lc" != "gps" ] && [ "$KEEPOUT_SITE" != "_" ]; then
  KEEPOUT_FILE="$WORKSPACE/src/eclipse_pkg/config/keepout_${KEEPOUT_SITE}_perimeter.geojson"
  if [ ! -f "$KEEPOUT_FILE" ]; then
    echo "keepout 맵이 없다: $KEEPOUT_FILE" >&2
    echo "있는 이름:" >&2
    ls -1 "$WORKSPACE/src/eclipse_pkg/config"/keepout_*_perimeter.geojson 2>/dev/null \
      | sed -e 's#.*/keepout_##' -e 's#_perimeter.geojson##' >&2 || true
    exit 1
  fi
fi

stop_core >/dev/null 2>&1 || true

docker run -d --rm --name "$CONTAINER" \
  --network host --ipc host --privileged \
  "${TARS_DOCKER_ROS_LOG_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -e ENABLE_FOXGLOVE="$ENABLE_FOXGLOVE" \
  -e TIDE_GPS_TOPIC="$TIDE_GPS_TOPIC" \
  -e KEEPOUT_SITE="$KEEPOUT_SITE" \
  -e WATERLINE_ALPHA_OVERRIDE="${WATERLINE_ALPHA_OVERRIDE:-}" \
  -e TZ="${TZ:-Asia/Seoul}" \
  -e DATA_GO_KR_SERVICE_KEY="${DATA_GO_KR_SERVICE_KEY:-}" \
  -e DATA_GO_KR_TIDEBED_KEY="${DATA_GO_KR_TIDEBED_KEY:-}" \
  -v /dev:/dev \
  -v "$WORKSPACE":/workspaces/eclipse-test-2 \
  "$IMAGE" \
  bash -lc '
    cd /workspaces/eclipse-test-2

    if [ ! -f install_trt/setup.bash ]; then
      echo "install_trt/setup.bash not found; build the TRT install space first" >&2
      exit 1
    fi
    source install_trt/setup.bash

    LAUNCH_ARGS=(
      enable_foxglove:="$ENABLE_FOXGLOVE"
      tide_gps_topic:="$TIDE_GPS_TOPIC"
    )
    if [ -n "${KEEPOUT_SITE:-}" ]; then
      LAUNCH_ARGS+=(keepout_site:="$KEEPOUT_SITE")
    fi
    if [ -n "${WATERLINE_ALPHA_OVERRIDE:-}" ]; then
      LAUNCH_ARGS+=(waterline_alpha_override:="$WATERLINE_ALPHA_OVERRIDE")
    fi
    exec ros2 launch eclipse_pkg tars_autonomy.launch.py "${LAUNCH_ARGS[@]}"
  '

exec docker logs -f "$CONTAINER"
