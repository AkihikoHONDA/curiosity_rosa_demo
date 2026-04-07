#!/usr/bin/env bash
set -euo pipefail

# Avoid stale ROS graph cache from previous runs in the same environment.
ros2 daemon stop >/dev/null 2>&1 || true

cd /workspace/overlay_ws
set +u
source /opt/ros/*/setup.bash
set -u
set +u
source install/setup.bash
set -u
ros2 launch curiosity_rosa_demo demo.launch.py "$@"
