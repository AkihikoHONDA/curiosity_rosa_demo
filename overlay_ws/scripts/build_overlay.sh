#!/usr/bin/env bash
set -eo pipefail

cd /workspace/overlay_ws
set +u
source /opt/ros/*/setup.bash
set -u
colcon --log-base /tmp/colcon_log build
