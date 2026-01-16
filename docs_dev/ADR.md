# ADR (Architecture Decision Records)

## ADR-001: Extend base image for OpenCV/numpy

### Status
Accepted

### Context
The simulator image pipeline depends on OpenCV and numpy for decode/encode and overlay.
The base image `osrf/space-ros:curiosity_demo` does not include these Python packages.

### Decision
Provide a derived image with `python3-opencv` and `python3-numpy` installed.
The Dockerfile is `docker/Dockerfile.curiosity_demo_ext`.

Build and use it as follows:

```bash
docker build -f docker/Dockerfile.curiosity_demo_ext -t curiosity_demo_ext .
```

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  curiosity_demo_ext \
  bash -lc "source /opt/ros/*/setup.bash && cd /workspace/overlay_ws && pytest -q src/curiosity_rosa_demo/tests"
```

### Consequences
- Requires a local build step to run tests and the simulator image pipeline.
- Keeps host dependencies minimal and makes test runs reproducible.

### Notes
The simulator currently uses the `score_update_hz` parameter for both score updates
and image processing. Consider consolidating to a more general name (e.g.,
`sim_update_hz`) in a future change, with explicit migration.

## ADR-002: Adapter exclusivity and status fields

### Status
Accepted

### Context
The adapter must enforce exclusivity (mast open blocks movement) while still
allowing emergency stop. The status endpoint should provide minimal but useful
state for debugging and tools.

### Decision
- Do not block `/adapter/move_stop` when the mast is open.
- Include `rover_x` in `get_status` when available.
- Clear `last_error_reason` on successful operations.

### Consequences
- The adapter allows safe stop even while the mast is open.
- Status responses include more context for debugging.

## ADR-003: Install ROSA in a dedicated venv inside the test image

### Status
Accepted

### Context
Installing ROSA via pip into the base image triggered PEP 668 protections and
led to NumPy/OpenCV ABI mismatches when pip pulled NumPy 2.x. The base image
ships OpenCV built against NumPy 1.x, so mixing system packages and pip upgrades
caused `ImportError: numpy.core.multiarray failed to import` during tests.

### Decision
Create a Python venv at `/opt/rosa_venv` inside
`docker/Dockerfile.curiosity_demo_ext` and install ROSA there with `numpy<2`.
Test commands should activate the venv before running pytest.

### Consequences
- ROSA and its Python dependencies are isolated from system packages.
- Test runs require `source /opt/rosa_venv/bin/activate` when using the
  extended image.
