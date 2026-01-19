#!/usr/bin/env bash
set -eo pipefail

cd /workspace/overlay_ws
set +u
source /opt/ros/*/setup.bash
set -u
set +u
source install/setup.bash
set -u
ros2 launch curiosity_rosa_demo demo.launch.py "$@"
