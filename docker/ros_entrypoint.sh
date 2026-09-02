#!/bin/bash
set -e

sourced_ros=false
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  sourced_ros=true
fi

if [ -f /opt/ros/humble/install/setup.bash ]; then
  source /opt/ros/humble/install/setup.bash
  sourced_ros=true
fi

if [ "$sourced_ros" != true ]; then
  echo "ROS Humble setup.bash not found" >&2
  exit 1
fi

if [ -f /workspaces/eclipse-test-2/install_trt/setup.bash ]; then
  source /workspaces/eclipse-test-2/install_trt/setup.bash
elif [ -f /workspaces/eclipse-test-2/install/setup.bash ]; then
  source /workspaces/eclipse-test-2/install/setup.bash
elif [ -f /workspace/eclipse-test-2/install_trt/setup.bash ]; then
  source /workspace/eclipse-test-2/install_trt/setup.bash
elif [ -f /workspace/eclipse-test-2/install/setup.bash ]; then
  source /workspace/eclipse-test-2/install/setup.bash
fi

exec "$@"
