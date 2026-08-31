#!/usr/bin/env bash
# TARS full autonomy stack (single entry point):
#   description (GPS/IMU/EKF/motors) + RealSense + Nav2 + GPS commander + Foxglove
#   + optional YOLO TRT (detached sidecar container)
#
# Usage:
#   ./scripts/run_jetson_autonomy.sh                        # autonomy only
#   ENABLE_YOLO_TRT=true ./scripts/run_jetson_autonomy.sh   # autonomy + YOLO TRT
#   ENABLE_FOXGLOVE=false ./scripts/run_jetson_autonomy.sh
#   ./scripts/run_jetson_autonomy.sh jebu                   # keepout map name
#   ./scripts/run_jetson_autonomy.sh start jebu
#   KEEPOUT_SITE=test ./scripts/run_jetson_autonomy.sh
#   ./scripts/run_jetson_autonomy.sh stop                   # stop all
#   ./scripts/run_jetson_autonomy.sh status                 # show containers
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tars_host_ros_logs.sh
source "$SCRIPT_DIR/tars_host_ros_logs.sh"
tars_prepare_host_ros_logs
WORKSPACE="${ECLIPSE_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
IMAGE="${ECLIPSE_DOCKER_IMAGE:-eclipse-test-2:humble}"
CONTAINER="${ECLIPSE_CONTAINER_NAME:-eclipse_test3_autonomy_live}"
DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
ENABLE_YOLO="${ENABLE_YOLO:-false}"
ENABLE_FOXGLOVE="${ENABLE_FOXGLOVE:-true}"
# tide_watch GPS source. Default is the real receiver. Never point this
# at a fake publisher on /gps/fix — that contaminates EKF/navsat.
TIDE_GPS_TOPIC="${TIDE_GPS_TOPIC:-/gps/fix}"
# 구운 맵 이름만. config/keepout_<이름>_perimeter.geojson
KEEPOUT_SITE="${KEEPOUT_SITE:-jebu}"

# --- YOLO TRT sidecar ---
ENABLE_YOLO_TRT="${ENABLE_YOLO_TRT:-false}"
TRT_IMAGE="${ECLIPSE_TRT_IMAGE:-eclipse-test-2:humble-yolo-trt}"
YOLO_CONTAINER="${ECLIPSE_YOLO_CONTAINER:-eclipse_test3_yolo_trt}"
YOLO_MODEL="${YOLO_MODEL:-/workspaces/eclipse-test-2/weights/merged5_run1.engine}"
_raw_device="${YOLO_DEVICE:-0}"
if [[ "${_raw_device}" =~ ^[0-9]+$ ]]; then
  DEVICE="cuda:${_raw_device}"
else
  DEVICE="${_raw_device}"
fi
YOLO_IMGSZ="${YOLO_IMGSZ:-320}"
YOLO_FORCE_CPU_NMS="${YOLO_FORCE_CPU_NMS:-false}"
YOLO_INFER_RATE="${YOLO_INFER_RATE:-15}"
# Cap /yolo/debug_image publish Hz (0 = match camera). Live baseline ~27 Hz.
YOLO_DEBUG_RATE="${YOLO_DEBUG_RATE:-12}"
YOLO_ALLOWED_CLASSES="${YOLO_ALLOWED_CLASSES:-person,shell}"
YOLO_CLASS_NAMES="${YOLO_CLASS_NAMES:-person,shell}"
YOLO_CLASS_THRESHOLDS="${YOLO_CLASS_THRESHOLDS:-person:0.65,shell:0.2}"
YOLO_ENABLE_3D="${YOLO_ENABLE_3D:-true}"
YOLO_ENABLE_SURVEY="${YOLO_ENABLE_SURVEY:-true}"
YOLO_SURVEY_OUTPUT_DIR="${YOLO_SURVEY_OUTPUT_DIR:-/tars_logs/survey}"
YOLO_SURVEY_CELL_M="${YOLO_SURVEY_CELL_M:-5.0}"
YOLO_THRESHOLD="${YOLO_THRESHOLD:-0.2}"
REBUILD_PKG="${REBUILD_ECLIPSE_PKG:-false}"
H264_UDP_ENABLE="${H264_UDP_ENABLE:-false}"
H264_UDP_HOST="${H264_UDP_HOST:-127.0.0.1}"
H264_UDP_PORT="${H264_UDP_PORT:-5600}"
H264_BITRATE="${H264_BITRATE:-3000000}"
H264_FPS="${H264_FPS:-30}"
DEBUG_MAX_WIDTH="${DEBUG_MAX_WIDTH:-320}"
DEBUG_MAX_HEIGHT="${DEBUG_MAX_HEIGHT:-180}"
JPEG_QUALITY="${JPEG_QUALITY:-65}"

# 조석 API 키: 워크스페이스가 아니라 호스트 env 파일에서만 읽는다.
TIDE_ENV_FILE="${TIDE_ENV_FILE:-$HOME/.config/tars/data_go_kr.env}"
if [ -f "$TIDE_ENV_FILE" ]; then
  set -a
  # shellcheck disable=SC1090
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

# ---------- subcommands ----------
ROSBRIDGE_CONTAINER="${ECLIPSE_ROSBRIDGE_CONTAINER:-eclipse_test3_rosbridge}"

# stop autonomy + yolo only (preserves rosbridge WebSocket connection)
stop_core() {
  docker rm -f "$CONTAINER" "$YOLO_CONTAINER" >/dev/null 2>&1 || true
  echo "stopped: $CONTAINER $YOLO_CONTAINER"
}

# preserve rosbridge WebSocket connection so web UI never loses connection
stop_all() {
  docker rm -f "$CONTAINER" "$YOLO_CONTAINER" >/dev/null 2>&1 || true
  echo "stopped autonomy nodes: $CONTAINER $YOLO_CONTAINER (ROSBridge WebSocket preserved)"
}

status_all() {
  echo "ROS_DOMAIN_ID=$DOMAIN_ID"
  docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' \
    | { head -n 1; grep -E "${CONTAINER}|${YOLO_CONTAINER}|${ROSBRIDGE_CONTAINER}" || true; }
}

case "$CMD" in
  stop)   stop_all;   exit 0 ;;
  status) status_all; exit 0 ;;
  start) ;;
  *) echo "usage: $0 [start|stop|status]" >&2; exit 2 ;;
esac

# ---------- preflight ----------
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
echo "enable_yolo:     $ENABLE_YOLO"
echo "enable_foxglove: $ENABLE_FOXGLOVE"
echo "enable_yolo_trt: $ENABLE_YOLO_TRT"
echo "keepout_site:    $KEEPOUT_SITE"
echo "ROS_LOG_DIR host: $TARS_HOST_LOG_DIR"
KEEPOUT_FILE="$WORKSPACE/src/eclipse_pkg/config/keepout_${KEEPOUT_SITE}_perimeter.geojson"
if [ ! -f "$KEEPOUT_FILE" ]; then
  echo "keepout 맵이 없다: $KEEPOUT_FILE" >&2
  echo "있는 이름:" >&2
  ls -1 "$WORKSPACE/src/eclipse_pkg/config"/keepout_*_perimeter.geojson 2>/dev/null \
    | sed -e 's#.*/keepout_##' -e 's#_perimeter.geojson##' >&2 || true
  echo "먼저 개발 PC에서 --install 하고 젯슨에 그 geojson 을 넘겨라." >&2
  exit 1
fi
if [ "$ENABLE_YOLO_TRT" = true ] && [[ "$YOLO_MODEL" == *merged5* ]] && [ -z "$YOLO_CLASS_NAMES" ]; then
  echo "WARN: merged5 engine without YOLO_CLASS_NAMES=person,shell — class 1 becomes bicycle and is dropped when allowed=person" >&2
fi

# Containers run detached. Stop with './scripts/run_jetson_autonomy.sh stop'.

# Automatically stop old autonomy/yolo containers if running (preserve rosbridge)
stop_core >/dev/null 2>&1 || true

# ---------- YOLO TRT sidecar (detached) ----------
if [ "$ENABLE_YOLO_TRT" = true ]; then
  if [ ! -f "${WORKSPACE}/weights/merged5_run1.engine" ] \
    && [ "$YOLO_MODEL" = "/workspaces/eclipse-test-2/weights/merged5_run1.engine" ]; then
    echo "engine missing: ${WORKSPACE}/weights/merged5_run1.engine" >&2
    exit 1
  fi

  docker rm -f "$YOLO_CONTAINER" >/dev/null 2>&1 || true
  sleep 1

  echo "starting YOLO TRT sidecar: $YOLO_CONTAINER"
  echo "  model: $YOLO_MODEL  device: $DEVICE  imgsz: $YOLO_IMGSZ"

  docker run -d --name "$YOLO_CONTAINER" \
    --runtime nvidia --network host --ipc host --privileged \
    "${TARS_DOCKER_ROS_LOG_ARGS[@]}" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/eclipse-test-2/docker/fastdds_udp_only.xml \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e YOLO_MODEL="$YOLO_MODEL" \
    -e YOLO_DEVICE="$DEVICE" \
    -e YOLO_IMGSZ="$YOLO_IMGSZ" \
    -e YOLO_FORCE_CPU_NMS="$YOLO_FORCE_CPU_NMS" \
    -e YOLO_INFER_RATE="$YOLO_INFER_RATE" \
    -e YOLO_DEBUG_RATE="$YOLO_DEBUG_RATE" \
    -e REBUILD_ECLIPSE_PKG="$REBUILD_PKG" \
    -e YOLO_ALLOWED_CLASSES="$YOLO_ALLOWED_CLASSES" \
    -e YOLO_CLASS_NAMES="$YOLO_CLASS_NAMES" \
    -e YOLO_CLASS_THRESHOLDS="$YOLO_CLASS_THRESHOLDS" \
    -e YOLO_ENABLE_3D="$YOLO_ENABLE_3D" \
    -e YOLO_ENABLE_SURVEY="$YOLO_ENABLE_SURVEY" \
    -e YOLO_SURVEY_OUTPUT_DIR="$YOLO_SURVEY_OUTPUT_DIR" \
    -e YOLO_SURVEY_CELL_M="$YOLO_SURVEY_CELL_M" \
    -e YOLO_THRESHOLD="$YOLO_THRESHOLD" \
    -e H264_UDP_ENABLE="$H264_UDP_ENABLE" \
    -e H264_UDP_HOST="$H264_UDP_HOST" \
    -e H264_UDP_PORT="$H264_UDP_PORT" \
    -e H264_BITRATE="$H264_BITRATE" \
    -e H264_FPS="$H264_FPS" \
    -e DEBUG_MAX_WIDTH="$DEBUG_MAX_WIDTH" \
    -e DEBUG_MAX_HEIGHT="$DEBUG_MAX_HEIGHT" \
    -e JPEG_QUALITY="$JPEG_QUALITY" \
    -v /dev:/dev \
    -v "$WORKSPACE":/workspaces/eclipse-test-2 \
    "$TRT_IMAGE" \
    bash -lc '
      set -euo pipefail
      set +u
      if [ -f /opt/ros/humble/install/setup.bash ]; then
        source /opt/ros/humble/install/setup.bash
      elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
      else
        echo "ROS Humble setup.bash not found" >&2; exit 1
      fi
      set -u

      python3 -c "import torch; print(\"torch\", torch.__version__, \"cuda\", torch.cuda.is_available()); assert torch.cuda.is_available(), \"cuda false\""
      if ! python3 -c "import numpy as np; v=np.__version__; print(\"numpy\", v); raise SystemExit(0 if v.startswith(\"1.23.\") else 1)"; then
        python3 -m pip install --quiet --timeout 60 "numpy==1.23.5" || true
      fi
      python3 -c "import numpy as np; print(\"numpy\", np.__version__); from ultralytics import YOLO; print(\"ultralytics-ok\")"
      if ! python3 -c "import torch; from torchvision.ops import nms; b=torch.zeros((1,4)).cuda(); sc=torch.zeros((1,)).cuda(); nms(b,sc,0.5)" 2>/dev/null; then
        echo "installing fixed torchvision wheel (CUDA NMS for SM 7.2)"
        python3 -m pip install --quiet --no-deps --force-reinstall /workspaces/eclipse-test-2/weights/tvwheel/torchvision-0.15.1a0-cp38-cp38-linux_aarch64.whl || true
      fi


      cd /workspaces/eclipse-test-2
      BUILD_BASE="${COLCON_BUILD_BASE:-build_trt}"
      INSTALL_BASE="${COLCON_INSTALL_BASE:-install_trt}"
      if [ "${REBUILD_ECLIPSE_PKG}" = true ] || [ ! -f "${INSTALL_BASE}/setup.bash" ]; then
        python3 -m pip install --quiet --upgrade \
          "packaging>=23,<26" "setuptools>=65,<80" "importlib-metadata>=6,<8" || true
        set +u
        colcon build --packages-up-to eclipse_pkg --symlink-install \
          --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE"
        source "${INSTALL_BASE}/setup.bash"
        set -u
      else
        set +u
        source "${INSTALL_BASE}/setup.bash"
        set -u
      fi

      if ! ros2 pkg executables eclipse_pkg | grep -qx "eclipse_pkg yolo_detect_node"; then
        echo "yolo_detect_node missing after build" >&2; exit 1
      fi

      echo "launching YOLO TRT (live) model=${YOLO_MODEL} device=${YOLO_DEVICE}"
      # ros2 launch rejects empty "name:=" values. class_names/class_thresholds
      # default empty so COCO engine names are used unless the caller sets them.
      LAUNCH_ARGS=(
        model:="${YOLO_MODEL}"
        device:="${YOLO_DEVICE}"
        imgsz:="${YOLO_IMGSZ}"
        force_cpu_nms:="${YOLO_FORCE_CPU_NMS}"
        infer_rate:="${YOLO_INFER_RATE}"
        debug_rate:="${YOLO_DEBUG_RATE}"
        enable_3d:="${YOLO_ENABLE_3D}"
        enable_survey:="${YOLO_ENABLE_SURVEY}"
        survey_output_dir:="${YOLO_SURVEY_OUTPUT_DIR}"
        survey_cell_m:="${YOLO_SURVEY_CELL_M}"
        allowed_classes:="${YOLO_ALLOWED_CLASSES}"
        threshold:="${YOLO_THRESHOLD}"
        h264_udp_enable:="${H264_UDP_ENABLE}"
        h264_udp_host:="${H264_UDP_HOST}"
        h264_udp_port:="${H264_UDP_PORT}"
        h264_bitrate:="${H264_BITRATE}"
        h264_fps:="${H264_FPS}"
        debug_max_width:="${DEBUG_MAX_WIDTH}"
        debug_max_height:="${DEBUG_MAX_HEIGHT}"
        jpeg_quality:="${JPEG_QUALITY}"
      )
      if [ -n "${YOLO_CLASS_NAMES:-}" ]; then
        LAUNCH_ARGS+=(class_names:="${YOLO_CLASS_NAMES}")
      fi
      if [ -n "${YOLO_CLASS_THRESHOLDS:-}" ]; then
        LAUNCH_ARGS+=(class_thresholds:="${YOLO_CLASS_THRESHOLDS}")
      fi
      exec ros2 launch eclipse_pkg yolo_detect.launch.py "${LAUNCH_ARGS[@]}"
    '

  echo "YOLO TRT sidecar started (detached): $YOLO_CONTAINER"
  echo "  logs:   docker logs -f $YOLO_CONTAINER"
fi

# ---------- main autonomy container (detached background) ----------
# TRT 사이드카가 YOLO를 담당하면 메인 런치의 YOLO 중복 기동을 막는다.
MAIN_ENABLE_YOLO="$ENABLE_YOLO"
if [ "$ENABLE_YOLO_TRT" = "true" ]; then MAIN_ENABLE_YOLO="false"; fi
docker run -d --rm --name "$CONTAINER" \
  --network host --ipc host --privileged \
  "${TARS_DOCKER_ROS_LOG_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -e ENABLE_YOLO="$MAIN_ENABLE_YOLO" \
  -e ENABLE_FOXGLOVE="$ENABLE_FOXGLOVE" \
  -e TIDE_GPS_TOPIC="$TIDE_GPS_TOPIC" \
  -e KEEPOUT_SITE="$KEEPOUT_SITE" \
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

    exec ros2 launch eclipse_pkg tars_autonomy.launch.py \
      enable_yolo:="$ENABLE_YOLO" \
      enable_foxglove:="$ENABLE_FOXGLOVE" \
      tide_gps_topic:="$TIDE_GPS_TOPIC" \
      keepout_site:="$KEEPOUT_SITE"
  '

exec docker logs -f "$CONTAINER"
