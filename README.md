# Curiosity ROSA Demo (overlay)

This repository provides an overlay package for the Space ROS Curiosity demo.
It adds a simulator node for light scoring and capture, an adapter, and tooling
for an LLM-driven demo.

## Prerequisites
- Docker (for `osrf/space-ros:curiosity_demo`)
- Colcon inside the demo container

## Build
```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  osrf/space-ros:curiosity_demo \
  bash -lc "source /opt/ros/*/setup.bash && cd /workspace/overlay_ws && colcon build"
```

If you use the extended image, the build command is the same:

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  curiosity_demo_ext \
  bash -lc "source /opt/ros/*/setup.bash && cd /workspace/overlay_ws && colcon build"
```

## Test
If you need OpenCV/numpy, build the extended image first:

```bash
docker build -f docker/Dockerfile.curiosity_demo_ext -t curiosity_demo_ext .
```

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  curiosity_demo_ext \
  bash -lc "source /opt/ros/*/setup.bash && source /opt/rosa_venv/bin/activate && cd /workspace/overlay_ws && pytest -q src/curiosity_rosa_demo/tests"
```

## Manual smoke (CaptureAndScore)
After launching the simulator node:

```bash
ros2 service call /capture_and_score curiosity_rosa_demo/srv/CaptureAndScore "{}"
```
