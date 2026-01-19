#!/usr/bin/env bash
set -eo pipefail

cd /workspace/overlay_ws
set +u
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
source /opt/rosa_venv/bin/activate
set -u
ros2 run curiosity_rosa_demo agent_node