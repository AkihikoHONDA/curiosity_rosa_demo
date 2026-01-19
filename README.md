# Curiosity - ROSA Demo

## Package Description
This repository provides an overlay package for the Space ROS Curiosity demo.
It adds a simulator node for light scoring and capture, an adapter, a visualizer,
and an LLM-driven agent console.

## Environment
- OS: WSL2 Ubuntu on Windows (assumed)
- Container runtime: Docker
- OpenAI API Key (set `OPENAI_API_KEY`)
- Base image build: `osrf/space-ros:curiosity_demo` is not distributed; build it from `https://github.com/space-ros/demos.git` by running `./build.sh` in `demos/curiosity_rover`

## Installation
1) Clone this repository.
2) Build the Curiosity demo base image (required, outside this repo):

```bash
git clone https://github.com/space-ros/demos.git
cd demos/curiosity_rover
./build.sh
```

3) Prepare `.env` for Docker Compose:

```bash
cp .env.example .env
```

Or generate it interactively:

```bash
chmod +x overlay_ws/scripts/gen_env.sh
./overlay_ws/scripts/gen_env.sh
```

## Quick Start (Docker Compose)
1) Build the overlay workspace:

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/build_overlay.sh
```

2) Launch the demo nodes (simulator/adapter/visualizer, RViz optional):

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/run_demo.sh
```

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/run_demo.sh use_rviz:=true
```

3) Start the agent in a separate terminal:

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/run_agent.sh
```

If the scripts are not executable, run:

```bash
docker compose run --rm curiosity_demo bash -lc "chmod +x /workspace/overlay_ws/scripts/*.sh"
```

## Minimal Smoke Test (T16)
Run these after the demo nodes are up:

I/F checks:
```bash
ros2 service list | grep -E "capture_and_score|/adapter/"
ros2 topic list | grep -E "/capture/image_raw|/trace/events"
```

Observation:
```bash
ros2 service call /capture_and_score curiosity_rosa_demo/srv/CaptureAndScore "{}"
```

Mast rotate:
```bash
ros2 service call /adapter/mast_rotate std_srvs/srv/Trigger "{}"
```

Move (after mast close if needed):
```bash
ros2 service call /adapter/mast_close std_srvs/srv/Trigger "{}"
ros2 service call /adapter/move_forward std_srvs/srv/Trigger "{}"
```

Exclusivity check:
```bash
ros2 service call /adapter/mast_open std_srvs/srv/Trigger "{}"
ros2 service call /adapter/move_forward std_srvs/srv/Trigger "{}"
# Expect: success=false, message="Need to close mast"
```

Trace check:
```bash
ros2 topic echo /trace/events --once
```

## Artifacts (trace/image)
Artifacts are saved under `overlay_ws/artifacts`.

Save trace to `overlay_ws/artifacts/trace.jsonl` (Ctrl+C to stop):
```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/save_trace.sh
```

Save one capture image (PPM):
```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/save_capture.sh
```

## LLM Agent Commands
You can give natural language instructions. These are the predefined console commands:
- `:help` show help
- `:cap` capture and score once
- `:status` show rover status (`:status llm` to explain)
- `:nudge` move forward briefly
- `:mast_open` mast open
- `:mast_close` mast close
- `:mast_rotate` mast rotate
- `:demo` run the demo prompt template
- `:quit` exit

LLM tools exposed to the agent:
- `capture_and_score`
- `mast_open`, `mast_close`, `mast_rotate`
- `move_nudge` (forward for a short, fixed duration; default 20.0s in `config/thresholds.yaml`)
- `get_status`

Notes:
- Initial mast state is treated as open; call `mast_close` before moving.
- If the mast is closed, `capture_and_score` fails with `Mast is closed`.

## Manual Run (docker run)
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
  -e XDG_RUNTIME_DIR=/tmp/xdg_runtime \
  -e RVIZ_CONFIG_DIR=/tmp/rviz2 \
  -e XAUTHORITY=$XAUTHORITY \
  -v "$XAUTHORITY:$XAUTHORITY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  --name curiosity_demo \
  curiosity_demo_ext \
  bash
```

Note: We run containers as the host user (non-root) to avoid root-owned files on
the host. The extended image sets `ROS_LOG_DIR=/tmp/ros_log` by default.

### Manual launch (inside the container)
Launch all demo nodes from this package:

```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
ros2 launch curiosity_rosa_demo demo.launch.py
```

Agent (ROSA venv required, separate terminal):

```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
source /opt/rosa_venv/bin/activate
ros2 run curiosity_rosa_demo agent_node
```

RViz:

```bash
source /opt/ros/*/setup.bash
rviz2 -d /workspace/overlay_ws/install/curiosity_rosa_demo/share/curiosity_rosa_demo/config/rviz.rviz
```

Note: `/capture/image_raw` is published with Reliable QoS, so RViz should show
it with default QoS settings.

## Troubleshooting
- Services missing: confirm the demo container is running and you sourced both `/opt/ros/*/setup.bash` and `/workspace/overlay_ws/install/setup.bash`.
- TF missing: verify the Curiosity demo is running and `/tf` is active.
- Image missing: call `/capture_and_score` to trigger capture (unless `debug:=true` is set).
- RViz not showing: check X11/WSLg settings (`DISPLAY`, `/tmp/.X11-unix`) and RViz envs (`XDG_RUNTIME_DIR`, `RVIZ_CONFIG_DIR`).
- LLM key missing: set `OPENAI_API_KEY` and activate `/opt/rosa_venv/bin/activate` before running `agent_node`.

## Tests
Tests require OpenCV and the ROSA venv, so use the extended image and run pytest:

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  curiosity_demo_ext \
  bash -lc "source /opt/ros/*/setup.bash && source /opt/rosa_venv/bin/activate && cd /workspace/overlay_ws && pytest -q src/curiosity_rosa_demo/tests"
```
