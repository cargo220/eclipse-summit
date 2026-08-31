# Source from Jetson run wrappers. Do not execute.
# ROS node/launch logs go to a Jetson host folder instead of the
# container overlay (~/.ros/log), which --rm deletes.

tars_prepare_host_ros_logs() {
  if [ -d /home/tars ]; then
    TARS_HOST_LOG_DIR="${TARS_HOST_LOG_DIR:-/home/tars/tars_logs}"
  else
    TARS_HOST_LOG_DIR="${TARS_HOST_LOG_DIR:-${HOME}/tars_logs}"
  fi
  TARS_CONTAINER_LOG_DIR="${TARS_CONTAINER_LOG_DIR:-/tars_logs}"
  mkdir -p "$TARS_HOST_LOG_DIR"
  chmod 1777 "$TARS_HOST_LOG_DIR" 2>/dev/null || true
  TARS_DOCKER_ROS_LOG_ARGS=(
    -e ROS_LOG_DIR="$TARS_CONTAINER_LOG_DIR"
    -v "$TARS_HOST_LOG_DIR":"$TARS_CONTAINER_LOG_DIR"
    --log-driver json-file
    --log-opt max-size=50m
    --log-opt max-file=3
  )
}
