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
