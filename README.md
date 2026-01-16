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

## Manual smoke (Agent console + trace)
1) Launch the simulator and adapter nodes, then start the agent node.
2) In the agent console, run `:cap` and confirm a summary prints to stdout.
3) In another terminal, confirm trace output:

```bash
ros2 topic echo /trace/events
```

Success log example:
```
Tool: capture_and_score: ok
```

Example commands (inside the demo container, after `colcon build`):

Start a single demo container (use `docker exec` for additional terminals):
```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -e OPENAI_API_KEY=YOUR_API_KEY \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  --name curiosity_demo \
  curiosity_demo_ext \
  bash
```

If you want GUI apps (RViz) in the same container, start it with X11/WSLg
passthrough (adjust env paths for your host setup):
```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -e OPENAI_API_KEY=YOUR_API_KEY \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR \
  -e XAUTHORITY=$XAUTHORITY \
  -v "$XAUTHORITY:$XAUTHORITY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /run/user:/run/user:ro \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  --name curiosity_demo \
  curiosity_demo_ext \
  bash
```

Note: We run containers as the host user (non-root) to avoid root-owned files on
the host. The extended image sets `ROS_LOG_DIR=/tmp/ros_log` by default. Override
if needed.

Terminal A (simulator):
```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
ros2 run curiosity_rosa_demo simulator_node
```

Terminal B (adapter) - from host:
```bash
docker exec -it curiosity_demo bash
```
```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
ros2 run curiosity_rosa_demo adapter_node
```

Terminal C (agent console) - from host:
```bash
docker exec -it curiosity_demo bash
```
```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
source /opt/rosa_venv/bin/activate
ros2 run curiosity_rosa_demo agent_node
```

Terminal D (trace echo) - from host:
```bash
docker exec -it curiosity_demo bash
```
```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
ros2 topic echo /trace/events
```

Terminal E (visualizer node) - from host:
```bash
docker exec -it curiosity_demo bash
```
```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
ros2 run curiosity_rosa_demo visualizer_node
```

Terminal F (RViz) - from host:
```bash
docker exec -it curiosity_demo bash
```
```bash
source /opt/ros/*/setup.bash
rviz2 -d /workspace/overlay_ws/install/curiosity_rosa_demo/share/curiosity_rosa_demo/config/rviz.yaml
```
