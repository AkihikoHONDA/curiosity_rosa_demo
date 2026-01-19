#!/usr/bin/env bash
set -eo pipefail

output_dir="/workspace/overlay_ws/artifacts"
mkdir -p "$output_dir"

out_path="$output_dir/trace.jsonl"
echo "writing trace to $out_path (Ctrl+C to stop)"
ros2 topic echo /trace/events > "$out_path"