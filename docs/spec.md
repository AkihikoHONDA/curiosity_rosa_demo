# Curiosity ROSA Demo - Spec (for design context)

This document summarizes the current system so changes can be designed without
reading the whole codebase.

## Goals
- Demonstrate an LLM-driven rover that can improve image brightness using
  tool calls and feedback (capture score).
- Provide observable reasoning/trace output (console + `/trace/events`).

## Variant A (current)
Mast open/close is disabled to simplify behavior. Mast rotate remains.
- LLM tools: `capture_and_score`, `mast_rotate`, `move_nudge`, `get_status`
- Adapter exclusivity (`Need to close mast`) is disabled.
- `capture_and_score` does not depend on mast state.
- `get_status.mast_is_open` returns `null`.

## System Overview
Nodes:
- `simulator_node`:
  - Computes brightness from rover X (TF), darkens camera image, republishes.
  - Provides `/capture_and_score` (CaptureAndScore).
- `adapter_node`:
  - Wraps Curiosity services with `/adapter/*` Trigger services.
  - Variant A: no exclusivity; `/adapter/mast_rotate` only.
- `visualizer_node`:
  - Publishes markers from trace and shows bright zone.
- `agent_node`:
  - ROSA-based agent with tool calls and trace output.

## Directory Overview (coarse)
Top-level structure (avoid tight coupling to fine details):
- `overlay_ws/`: ROS2 overlay workspace
  - `src/curiosity_rosa_demo/`: package source
    - `curiosity_rosa_demo/`: runtime modules (agent/sim/adapter/viz/trace/tools/infra)
    - `config/`: YAML configs (topics/thresholds/prompts/tool_costs/rviz)
    - `launch/`, `srv/`, `tests/`
- `docker/`: Dockerfiles for demo images
- `docs_dev/`: requirements/design/tasks/contract
- `docs/`: summary specs (this file)

## Interfaces
Topics:
- `/capture/image_raw` (sensor_msgs/msg/Image)
- `/trace/events` (std_msgs/msg/String JSON)
- TF frames: `world_frame` and `base_frame` from `config/topics.yaml`

Services (Trigger):
- `/adapter/move_forward`, `/adapter/turn_left`, `/adapter/turn_right`,
  `/adapter/move_stop`, `/adapter/mast_rotate`, `/adapter/get_status`

Service (custom):
- `/capture_and_score` (curiosity_rosa_demo/srv/CaptureAndScore)
  - Returns: `ok`, `score`, `is_good`, `image_topic`, `stamp`, `debug`.

Curiosity services (Empty, wrapped by adapter):
- `/move_forward`, `/turn_left`, `/turn_right`, `/move_stop`, `/mast_rotate`

## LLM Tools
Implemented in `curiosity_rosa_demo/tools/tool_impl.py`:
- `capture_and_score`
  - Fails with `error_reason="Move in progress"` if `move_nudge` is running.
- `mast_rotate` (sleep ~6s before returning)
- `move_nudge`
  - Calls `/adapter/move_forward`, sleeps `move.nudge_duration_sec`,
    then `/adapter/move_stop`.
- `get_status` (reads `/adapter/get_status`)

Console commands:
- `:cap`, `:nudge`, `:mast_rotate`, `:status`, `:demo`, `:help`, `:quit`

## Configuration
`overlay_ws/src/curiosity_rosa_demo/config/*.yaml`:
- `topics.yaml`: topic/service names + TF frames
- `thresholds.yaml`:
  - `light_model.x_min`, `light_model.x_good`
  - `quality.score_threshold`
  - `move.nudge_duration_sec`
- `tool_costs.yaml`: tool costs used in logs/prompts
- `prompts.yaml`: ROSA system prompt + few-shot examples
- `rviz.rviz`: RViz preset (used by docs)

## Agent Parameters
ROS2 params on `agent_node`:
- `agent_streaming` (default true)
- `agent_verbose` (default true)
- `agent_rich_ui` (default true)
- `agent_max_iterations` (default 100)

## Trace Events
Published to `/trace/events` as JSON:
- `kind` in {OBSERVE, HYPOTHESIZE, DECIDE, ACT, RESULT, ERROR}
- tool name, ok/error_reason, score, message, data

## Prompts (current behavior)
- Uses capture score to decide next actions.
- Few-shot examples show multi-step tool use for bright images.
- Avoids internal threshold disclosure.
- Avoids tool overlap (no capture while moving).

## Tests
Run via Docker:
```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  curiosity_demo_ext \
  bash -lc "source /opt/ros/*/setup.bash && source /opt/rosa_venv/bin/activate && cd /workspace/overlay_ws && pytest -q src/curiosity_rosa_demo/tests"
```

## Known Limitations
- Variant A disables mast open/close exclusivity behavior.
- Brightness logic depends on TF availability; no TF means no score update.

## Data Flow
```
Curiosity Gazebo camera (/image_raw/compressed)
  -> simulator_node (darken + overlay score)
    -> /capture/image_raw (Image)

TF (odom -> base_footprint)
  -> simulator_node (LightModel score)
    -> CaptureAndScore service response

agent_node (ROSA)
  -> tools (capture_and_score / mast_rotate / move_nudge / get_status)
    -> adapter_node (/adapter/* Trigger)
      -> Curiosity services (/move_forward, /move_stop, /mast_rotate)

agent_node (trace)
  -> /trace/events (JSON)
    -> visualizer_node (MarkerArray)
```

## State Transitions (high level)
### Bright-Image Loop (Variant A)
```
Start
  -> capture_and_score
    -> is_good == true  -> Done (report success)
    -> is_good == false -> mast_rotate
        -> capture_and_score
            -> is_good == true -> Done
            -> is_good == false -> move_nudge
                -> capture_and_score
                    -> repeat up to 10 attempts
```

### Move/Capture Guard
```
move_nudge in progress
  -> capture_and_score returns ok=false, error_reason="Move in progress"
move_nudge done
  -> capture_and_score allowed
```

## TODO / Future Work
- Re-enable mast open/close (Variant B) and restore exclusivity logic.
- Remove temporary tool delays or replace with status-based readiness checks.
- Align LLM behavior with exclusivity (auto close before move, open before capture).
- Improve move/capture sequencing without heuristic guards (LLM-native planning).
- Add lightweight concurrency control in tools to avoid overlapping calls.
- Revisit brightness threshold (x_good) to match the original demo narrative.
- Expand tests to cover Variant B and exclusivity paths.

## Variant B (mast enabled, planned)
This is the intended/full variant when mast control is restored.
- Tools: `capture_and_score`, `mast_open`, `mast_close`, `mast_rotate`, `move_nudge`, `get_status`
- Adapter exclusivity enabled:
  - If mast is open, move requests fail with `Need to close mast`
- `capture_and_score` fails when mast is closed (`Mast is closed`)
- `get_status.mast_is_open` is boolean
- Expected loop:
  - capture -> rotate -> capture -> close -> move -> open -> capture (repeat)
