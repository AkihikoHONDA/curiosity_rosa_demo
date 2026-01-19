import json
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from curiosity_rosa_demo.tools import tool_impl

try:
    from curiosity_rosa_demo.srv import CaptureAndScore
except Exception:  # pragma: no cover - available after rosidl build
    CaptureAndScore = None


@pytest.fixture
def tool_node():
    rclpy.init()
    node = Node("test_tools_node")
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _config_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "config"


def _create_trigger_service(node: Node, name: str, handler):
    node.create_service(Trigger, name, handler)


def _create_capture_service(node: Node, handler):
    if CaptureAndScore is None:
        pytest.skip("CaptureAndScore type not generated")
    node.create_service(CaptureAndScore, "/capture_and_score", handler)


def test_trigger_tools_and_status(tool_node: Node):
    status_message = {"message": json.dumps({"mast_is_open": False})}

    def _trigger_handler(_request, response):
        if response is None:
            return response
        return response

    def _make_handler(name: str):
        def _handle(_request, response):
            if name == "move_nudge":
                response.success = False
                response.message = "Need to close mast"
                return response
            if name == "get_status":
                response.success = True
                response.message = status_message["message"]
                return response
            response.success = True
            response.message = ""
            return response
        return _handle

    _create_trigger_service(tool_node, "/adapter/mast_open", _make_handler("mast_open"))
    _create_trigger_service(tool_node, "/adapter/move_forward", _make_handler("move_nudge"))
    _create_trigger_service(tool_node, "/adapter/move_stop", _make_handler("move_stop"))
    _create_trigger_service(tool_node, "/adapter/get_status", _make_handler("get_status"))

    result = tool_impl.mast_open(tool_node, config_dir=_config_dir())
    assert result.ok is True
    assert result.data["cost"] == 2

    result = tool_impl.move_nudge(tool_node, config_dir=_config_dir())
    assert result.ok is False
    assert result.error_reason == "Need to close mast"
    assert result.data["cost"] == 5

    result = tool_impl.get_status(tool_node, config_dir=_config_dir())
    assert result.ok is True
    assert result.data["mast_is_open"] is False
    assert result.data["cost"] == 1

    status_message["message"] = "not-json"
    result = tool_impl.get_status(tool_node, config_dir=_config_dir())
    assert result.ok is True
    assert result.data["raw_message"] == "not-json"


def test_capture_and_score(tool_node: Node):
    capture_state = {"ok": True}

    def _capture_handler(_request, response):
        response.ok = capture_state["ok"]
        response.score = 0.75 if capture_state["ok"] else 0.1
        response.is_good = bool(capture_state["ok"])
        response.error_reason = "" if capture_state["ok"] else "image not received yet"
        response.image_topic = "/capture/image_raw"
        response.debug = "test"
        return response

    _create_capture_service(tool_node, _capture_handler)

    result = tool_impl.capture_and_score(tool_node, config_dir=_config_dir())
    assert result.ok is True
    assert result.data["score"] == pytest.approx(0.75, abs=1e-3)
    assert result.data["cost"] == 1

    capture_state["ok"] = False
    result = tool_impl.capture_and_score(tool_node, config_dir=_config_dir())
    assert result.ok is False
    assert result.error_reason == "image not received yet"
    assert result.data["score"] == pytest.approx(0.0, abs=1e-3)
    assert result.data["is_good"] is False
