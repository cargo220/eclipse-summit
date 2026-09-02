#!/usr/bin/env bash
# Robot description stack on Jetson (GPS/IMU/EKF/wheels).
#
# Do not run this at the same time as run_jetson_autonomy.sh — both start
# eclipse_test_controller.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ECLIPSE_WORKSPACE="${ECLIPSE_WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
export ECLIPSE_LAUNCH_FILE="description.launch.py"
export ECLIPSE_CONTAINER_NAME="${ECLIPSE_CONTAINER_NAME:-eclipse_test3_description_live}"

exec "$SCRIPT_DIR/run_jetson_launch.sh" "$@"
