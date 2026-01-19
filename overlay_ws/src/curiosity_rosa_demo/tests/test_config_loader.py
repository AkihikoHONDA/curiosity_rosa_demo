from pathlib import Path

import pytest

from curiosity_rosa_demo.infra.config_loader import load_all_configs


def _write_yaml(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")


def _write_minimal_configs(base_dir: Path, topics_override: str | None = None) -> None:
    topics = topics_override or (
        """
images:
  input_compressed: "/image_raw/compressed"
  output_capture_raw: "/capture/image_raw"

trace:
  events: "/trace/events"

tf:
  base_frame: "base_link"
  world_frame: "map"

services:
  capture_and_score:
    kind: "service"
    name: "/capture_and_score"
    type: "curiosity_rosa_demo/srv/CaptureAndScore"

adapter:
  move_forward: { kind: "service", name: "/adapter/move_forward", type: "std_srvs/srv/Trigger" }
  turn_left:    { kind: "service", name: "/adapter/turn_left",    type: "std_srvs/srv/Trigger" }
  turn_right:   { kind: "service", name: "/adapter/turn_right",   type: "std_srvs/srv/Trigger" }
  move_stop:    { kind: "service", name: "/adapter/move_stop",    type: "std_srvs/srv/Trigger" }
  mast_open:    { kind: "service", name: "/adapter/mast_open",    type: "std_srvs/srv/Trigger" }
  mast_close:   { kind: "service", name: "/adapter/mast_close",   type: "std_srvs/srv/Trigger" }
  mast_rotate:  { kind: "service", name: "/adapter/mast_rotate",  type: "std_srvs/srv/Trigger" }
  get_status:   { kind: "service", name: "/adapter/get_status",   type: "std_srvs/srv/Trigger" }

curiosity:
  move_forward: { kind: "service", name: "/move_forward", type: "std_srvs/srv/Empty" }
  turn_left:    { kind: "service", name: "/turn_left",    type: "std_srvs/srv/Empty" }
  turn_right:   { kind: "service", name: "/turn_right",   type: "std_srvs/srv/Empty" }
  move_stop:    { kind: "service", name: "/move_stop",    type: "std_srvs/srv/Empty" }
  mast_open:    { kind: "service", name: "/mast_open",    type: "std_srvs/srv/Empty" }
  mast_close:   { kind: "service", name: "/mast_close",   type: "std_srvs/srv/Empty" }
  mast_rotate:  { kind: "service", name: "/mast_rotate",  type: "std_srvs/srv/Empty" }

viz:
  marker_array: "/visualization_marker_array"
"""
    )
    _write_yaml(base_dir / "topics.yaml", topics)
    _write_yaml(
        base_dir / "thresholds.yaml",
        """
light_model:
  x_min: 0.0
  x_good: 5.0

quality:
  score_threshold: 0.8
""",
    )
    _write_yaml(
        base_dir / "tool_costs.yaml",
        """
tools:
  capture_and_score: 1
""",
    )
    _write_yaml(
        base_dir / "prompts.yaml",
        """
robot_system_prompts:
  embodiment_and_persona: "persona"
  critical_instructions: "critical"
  relevant_context: "context"
  nuance_and_assumptions: "nuance"

bootstrap:
  enabled: true
  text: "bootstrap"

memory:
  enabled: true
  max_events: 5

templates: {}
""",
    )
    _write_yaml(base_dir / "rviz.yaml", "fixed_frame: map\n")


def test_load_all_configs_minimal(tmp_path: Path) -> None:
    _write_minimal_configs(tmp_path)
    bundle = load_all_configs(config_dir=tmp_path)

    assert bundle.topics.images.input_compressed == "/image_raw/compressed"
    assert bundle.topics.adapter.move_forward.name == "/adapter/move_forward"
    assert bundle.thresholds.quality_score_threshold == 0.8
    assert bundle.tool_costs.tools["capture_and_score"] == 1
    assert bundle.prompts.bootstrap.enabled is True
    assert bundle.rviz.raw["fixed_frame"] == "map"


def test_missing_required_key_raises(tmp_path: Path) -> None:
    _write_minimal_configs(tmp_path)
    bad_topics = """
images:
  output_capture_raw: "/capture/image_raw"

trace:
  events: "/trace/events"

tf:
  base_frame: "base_link"
  world_frame: "map"

services:
  capture_and_score:
    kind: "service"
    name: "/capture_and_score"
    type: "curiosity_rosa_demo/srv/CaptureAndScore"

adapter:
  move_forward: { kind: "service", name: "/adapter/move_forward", type: "std_srvs/srv/Trigger" }
  turn_left:    { kind: "service", name: "/adapter/turn_left",    type: "std_srvs/srv/Trigger" }
  turn_right:   { kind: "service", name: "/adapter/turn_right",   type: "std_srvs/srv/Trigger" }
  move_stop:    { kind: "service", name: "/adapter/move_stop",    type: "std_srvs/srv/Trigger" }
  mast_open:    { kind: "service", name: "/adapter/mast_open",    type: "std_srvs/srv/Trigger" }
  mast_close:   { kind: "service", name: "/adapter/mast_close",   type: "std_srvs/srv/Trigger" }
  mast_rotate:  { kind: "service", name: "/adapter/mast_rotate",  type: "std_srvs/srv/Trigger" }
  get_status:   { kind: "service", name: "/adapter/get_status",   type: "std_srvs/srv/Trigger" }

curiosity:
  move_forward: { kind: "service", name: "/move_forward", type: "std_srvs/srv/Empty" }
  turn_left:    { kind: "service", name: "/turn_left",    type: "std_srvs/srv/Empty" }
  turn_right:   { kind: "service", name: "/turn_right",   type: "std_srvs/srv/Empty" }
  move_stop:    { kind: "service", name: "/move_stop",    type: "std_srvs/srv/Empty" }
  mast_open:    { kind: "service", name: "/mast_open",    type: "std_srvs/srv/Empty" }
  mast_close:   { kind: "service", name: "/mast_close",   type: "std_srvs/srv/Empty" }
  mast_rotate:  { kind: "service", name: "/mast_rotate",  type: "std_srvs/srv/Empty" }

viz:
  marker_array: "/visualization_marker_array"
"""
    _write_yaml(tmp_path / "topics.yaml", bad_topics)

    with pytest.raises(ValueError) as excinfo:
        load_all_configs(config_dir=tmp_path)

    message = str(excinfo.value)
    assert "topics.yaml" in message
    assert "images.input_compressed" in message


def test_tool_costs_negative_raises(tmp_path: Path) -> None:
    _write_minimal_configs(tmp_path)
    _write_yaml(
        tmp_path / "tool_costs.yaml",
        """
tools:
  capture_and_score: -1
""",
    )

    with pytest.raises(ValueError) as excinfo:
        load_all_configs(config_dir=tmp_path)

    message = str(excinfo.value)
    assert "tool_costs.yaml" in message
    assert "tools.capture_and_score" in message


def test_score_threshold_range_raises(tmp_path: Path) -> None:
    _write_minimal_configs(tmp_path)
    _write_yaml(
        tmp_path / "thresholds.yaml",
        """
light_model:
  x_min: 0.0
  x_good: 5.0

quality:
  score_threshold: 1.5
""",
    )

    with pytest.raises(ValueError) as excinfo:
        load_all_configs(config_dir=tmp_path)

    message = str(excinfo.value)
    assert "thresholds.yaml" in message
    assert "quality.score_threshold" in message
