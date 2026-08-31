#!/usr/bin/env bash
# Keep RealSense + TensorRT YOLO running (detached containers).
# Smoke counterpart: run_yolo_trt_docker_check.sh (exits after hz).
#
# Usage:
#   ./scripts/run_yolo_trt_live.sh           # start
#   ./scripts/run_yolo_trt_live.sh status
#   ./scripts/run_yolo_trt_live.sh stop
#   ./scripts/run_yolo_trt_live.sh logs      # follow YOLO logs
#
# Env (optional):
#   ROS_DOMAIN_ID=27
#   YOLO_DEVICE=0 | cuda:0
#   YOLO_IMGSZ=320
#   YOLO_FORCE_CPU_NMS=auto
#   REBUILD_ECLIPSE_PKG=false
#   START_REALSENSE=true
#   H264_UDP_ENABLE=true H264_UDP_HOST=10.10.1.19 H264_UDP_PORT=5600
#   H264_BITRATE=3000000 H264_FPS=30
#   DEBUG_MAX_WIDTH=320 DEBUG_MAX_HEIGHT=180  (0=no limit)
#   JPEG_QUALITY=65
#   YOLO_THRESHOLD=0.2
#   YOLO_ALLOWED_CLASSES=person,shell
#   YOLO_CLASS_NAMES=person,shell   # merged5 engine (class 1 is shell, not bicycle)
#   YOLO_CLASS_THRESHOLDS=person:0.65,shell:0.2
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tars_host_ros_logs.sh
source "$SCRIPT_DIR/tars_host_ros_logs.sh"
tars_prepare_host_ros_logs

WORKSPACE="${ECLIPSE_WORKSPACE:-/home/tars/ewooni_docker_6}"
RS_IMAGE="${ECLIPSE_RS_IMAGE:-eclipse-test-2:humble-yolo}"
TRT_IMAGE="${ECLIPSE_TRT_IMAGE:-eclipse-test-2:humble-yolo-trt}"
RS_CONTAINER="${ECLIPSE_RS_CONTAINER:-eclipse_test3_rs_trt}"
YOLO_CONTAINER="${ECLIPSE_YOLO_CONTAINER:-eclipse_test3_yolo_trt}"
DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
MODEL="${YOLO_MODEL:-/workspaces/eclipse-test-2/weights/merged5_run1.engine}"
_raw_device="${YOLO_DEVICE:-0}"
if [[ "${_raw_device}" =~ ^[0-9]+$ ]]; then
  DEVICE="cuda:${_raw_device}"
else
  DEVICE="${_raw_device}"
fi
IMGSZ="${YOLO_IMGSZ:-320}"
FORCE_CPU_NMS="${YOLO_FORCE_CPU_NMS:-auto}"
START_REALSENSE="${START_REALSENSE:-true}"
COLOR_PROFILE="${REALSENSE_COLOR_PROFILE:-424x240x30}"
DEPTH_PROFILE="${REALSENSE_DEPTH_PROFILE:-480x270x30}"
REBUILD_PKG="${REBUILD_ECLIPSE_PKG:-false}"
ALLOWED_CLASSES="${YOLO_ALLOWED_CLASSES:-person,shell}"
CLASS_NAMES="${YOLO_CLASS_NAMES:-person,shell}"
CLASS_THRESHOLDS="${YOLO_CLASS_THRESHOLDS:-person:0.65,shell:0.2}"
ENABLE_3D="${YOLO_ENABLE_3D:-true}"
ENABLE_SURVEY="${YOLO_ENABLE_SURVEY:-true}"
SURVEY_OUTPUT_DIR="${YOLO_SURVEY_OUTPUT_DIR:-/tars_logs/survey}"
SURVEY_CELL_M="${YOLO_SURVEY_CELL_M:-5.0}"
THRESHOLD="${YOLO_THRESHOLD:-0.2}"
H264_UDP_ENABLE="${H264_UDP_ENABLE:-false}"
# Launch args cannot be empty (ros2 rejects host:=). Dummy when disabled.
H264_UDP_HOST="${H264_UDP_HOST:-127.0.0.1}"
H264_UDP_PORT="${H264_UDP_PORT:-5600}"
H264_BITRATE="${H264_BITRATE:-3000000}"
H264_FPS="${H264_FPS:-30}"
DEBUG_MAX_WIDTH="${DEBUG_MAX_WIDTH:-320}"
DEBUG_MAX_HEIGHT="${DEBUG_MAX_HEIGHT:-180}"
JPEG_QUALITY="${JPEG_QUALITY:-65}"
CMD="${1:-start}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found" >&2
  exit 1
fi

status_cmd() {
  echo "ROS_DOMAIN_ID=$DOMAIN_ID"
  docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' \
    | { head -n 1; grep -E "${RS_CONTAINER}|${YOLO_CONTAINER}" || true; }
  if docker ps --format '{{.Names}}' | grep -qx "$YOLO_CONTAINER"; then
    echo "--- yolo log (tail) ---"
    docker logs --tail 20 "$YOLO_CONTAINER" 2>&1 || true
  fi
}

stop_cmd() {
  docker rm -f "$YOLO_CONTAINER" "$RS_CONTAINER" >/dev/null 2>&1 || true
  echo "stopped: $YOLO_CONTAINER $RS_CONTAINER"
}

logs_cmd() {
  if ! docker ps --format '{{.Names}}' | grep -qx "$YOLO_CONTAINER"; then
    echo "YOLO container not running: $YOLO_CONTAINER" >&2
    exit 1
  fi
  exec docker logs -f "$YOLO_CONTAINER"
}

case "$CMD" in
  stop)
    stop_cmd
    exit 0
    ;;
  status)
    status_cmd
    exit 0
    ;;
  logs)
    logs_cmd
    ;;
  start) ;;
  *)
    echo "usage: $0 [start|stop|status|logs]" >&2
    exit 2
    ;;
esac

if [ ! -d "$WORKSPACE" ]; then
  echo "workspace not found: $WORKSPACE" >&2
  exit 1
fi

if [ ! -f "${WORKSPACE}/weights/merged5_run1.engine" ] \
  && [ "$MODEL" = "/workspaces/eclipse-test-2/weights/merged5_run1.engine" ]; then
  echo "engine missing: ${WORKSPACE}/weights/merged5_run1.engine" >&2
  exit 1
fi

# Replace previous live/smoke containers of the same names.
docker rm -f "$YOLO_CONTAINER" "$RS_CONTAINER" >/dev/null 2>&1 || true
sleep 1

echo "workspace: $WORKSPACE"
echo "rs_image: $RS_IMAGE  trt_image: $TRT_IMAGE"
echo "model: $MODEL device: $DEVICE imgsz: $IMGSZ force_cpu_nms: $FORCE_CPU_NMS"
echo "ROS_DOMAIN_ID: $DOMAIN_ID allowed_classes: $ALLOWED_CLASSES class_names: ${CLASS_NAMES:-<coco-fallback>} class_thresholds: ${CLASS_THRESHOLDS:-<global>} enable_3d: $ENABLE_3D threshold: $THRESHOLD"
if [[ "$MODEL" == *merged5* ]] && [ -z "$CLASS_NAMES" ]; then
  echo "WARN: merged5 engine without YOLO_CLASS_NAMES=person,shell — class 1 becomes bicycle and is dropped when allowed=person" >&2
fi
echo "rebuild_pkg: $REBUILD_PKG"
echo "ROS_LOG_DIR host: $TARS_HOST_LOG_DIR"
echo "h264_udp: enable=$H264_UDP_ENABLE host=$H264_UDP_HOST port=$H264_UDP_PORT bitrate=$H264_BITRATE"
echo "debug_jpeg: max=${DEBUG_MAX_WIDTH}x${DEBUG_MAX_HEIGHT} q=${JPEG_QUALITY} (compressed only)"

if [ "$START_REALSENSE" = true ]; then
  docker run -d --name "$RS_CONTAINER" \
    --network host --ipc host --privileged \
    "${TARS_DOCKER_ROS_LOG_ARGS[@]}" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/eclipse-test-2/docker/fastdds_udp_only.xml \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e REALSENSE_COLOR_PROFILE="$COLOR_PROFILE" \
    -e REALSENSE_DEPTH_PROFILE="$DEPTH_PROFILE" \
    -v /dev:/dev \
    -v "$WORKSPACE":/workspaces/eclipse-test-2 \
    "$RS_IMAGE" \
    bash -lc '
      set +u
      source /opt/ros/humble/setup.bash
      cd /workspaces/eclipse-test-2
      if [ -f install/setup.bash ]; then source install/setup.bash; fi
      set -u
      exec ros2 launch eclipse_pkg realsense_d435.launch.py \
        color_profile:="${REALSENSE_COLOR_PROFILE}" \
        depth_profile:="${REALSENSE_DEPTH_PROFILE}"
    ' >/tmp/rs_trt_live.log 2>&1

  echo "waiting for color topic..."
  found_color=false
  for _ in $(seq 1 50); do
    if docker run --rm --network host \
      -e ROS_DOMAIN_ID="$DOMAIN_ID" \
      "$RS_IMAGE" \
      bash -lc 'set +u; source /opt/ros/humble/setup.bash; set -u; \
        timeout 3 ros2 topic list 2>/dev/null | grep -qx /camera/camera/color/image_raw' \
      >/dev/null 2>&1; then
      found_color=true
      break
    fi
    if ! docker ps --format '{{.Names}}' | grep -qx "$RS_CONTAINER"; then
      docker logs "$RS_CONTAINER" 2>&1 | tail -80 >&2 || true
      exit 1
    fi
    sleep 1
  done
  if [ "$found_color" != true ]; then
    docker logs "$RS_CONTAINER" 2>&1 | tail -80 >&2 || true
    echo "color topic not found" >&2
    exit 1
  fi
  echo "color topic up"
fi

docker run -d --name "$YOLO_CONTAINER" \
  --runtime nvidia --network host --ipc host --privileged \
  "${TARS_DOCKER_ROS_LOG_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/eclipse-test-2/docker/fastdds_udp_only.xml \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e YOLO_MODEL="$MODEL" \
  -e YOLO_DEVICE="$DEVICE" \
  -e YOLO_IMGSZ="$IMGSZ" \
  -e YOLO_FORCE_CPU_NMS="$FORCE_CPU_NMS" \
  -e REBUILD_ECLIPSE_PKG="$REBUILD_PKG" \
  -e YOLO_ALLOWED_CLASSES="$ALLOWED_CLASSES" \
  -e YOLO_CLASS_NAMES="$CLASS_NAMES" \
  -e YOLO_CLASS_THRESHOLDS="$CLASS_THRESHOLDS" \
  -e YOLO_ENABLE_3D="$ENABLE_3D" \
  -e YOLO_ENABLE_SURVEY="$ENABLE_SURVEY" \
  -e YOLO_SURVEY_OUTPUT_DIR="$SURVEY_OUTPUT_DIR" \
  -e YOLO_SURVEY_CELL_M="$SURVEY_CELL_M" \
  -e YOLO_THRESHOLD="$THRESHOLD" \
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
      echo "ROS Humble setup.bash not found" >&2
      exit 1
    fi
    set -u

    python3 -c "import torch; print(\"torch\", torch.__version__, \"cuda\", torch.cuda.is_available()); assert torch.cuda.is_available(), \"cuda false\""
    # Prefer image-pinned numpy==1.23.5 (Dockerfile). Runtime pip only if missing/wrong.
    if ! python3 -c "import numpy as np; v=np.__version__; print(\"numpy\", v); raise SystemExit(0 if v.startswith(\"1.23.\") else 1)"; then
      python3 -m pip install --quiet --timeout 60 "numpy==1.23.5" || true
    fi
    python3 -c "import numpy as np; print(\"numpy\", np.__version__); from ultralytics import YOLO; print(\"ultralytics-ok\")"

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
      echo "yolo_detect_node missing after build" >&2
      exit 1
    fi

    echo "launching YOLO TRT (live) model=${YOLO_MODEL} device=${YOLO_DEVICE}"
    # ros2 launch rejects empty "name:=" values. Omit optional overrides when unset.
    LAUNCH_ARGS=(
      model:="${YOLO_MODEL}"
      device:="${YOLO_DEVICE}"
      imgsz:="${YOLO_IMGSZ}"
      force_cpu_nms:="${YOLO_FORCE_CPU_NMS}"
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

echo "started (detached):"
echo "  RS:   $RS_CONTAINER"
echo "  YOLO: $YOLO_CONTAINER"
echo "view (one subscriber only — do not run rqt + Foxglove together):"
echo "  preferred: ./scripts/foxglove_yolo_debug.sh"
echo "  topic: /yolo/debug_image/compressed  (max ${DEBUG_MAX_WIDTH}x${DEBUG_MAX_HEIGHT} q=${JPEG_QUALITY})"
if [ "$H264_UDP_ENABLE" = true ] && [ -n "$H264_UDP_HOST" ]; then
  echo "  h264 udp: ${H264_UDP_HOST}:${H264_UDP_PORT}  (laptop: scripts/view_yolo_h264.sh)"
fi
echo "logs:   $0 logs"
echo "status: $0 status"
echo "stop:   $0 stop"
