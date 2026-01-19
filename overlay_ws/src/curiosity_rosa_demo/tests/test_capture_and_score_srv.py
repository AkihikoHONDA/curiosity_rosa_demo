from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from curiosity_rosa_demo.domain.models import LightScore
from curiosity_rosa_demo.sim.simulator_node import SimulatorNode

try:
    from curiosity_rosa_demo.srv import CaptureAndScore
except Exception:  # pragma: no cover - available after rosidl build
    CaptureAndScore = None


@pytest.fixture
def ros_nodes():
    rclpy.init()
    config_dir = Path(__file__).resolve().parents[1] / "config"
    sim_node = SimulatorNode(config_dir=config_dir)
    client_node = Node("test_capture_client")
    try:
        yield sim_node, client_node
    finally:
        sim_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def _call_service(client_node: Node, sim_node: SimulatorNode):
    if CaptureAndScore is None:
        pytest.skip("CaptureAndScore service type not generated")
    service_name = sim_node._topics.services.capture_and_score.name
    client = client_node.create_client(CaptureAndScore, service_name)
    if not client.wait_for_service(timeout_sec=1.0):
        pytest.skip("CaptureAndScore service not available")
    future = client.call_async(CaptureAndScore.Request())
    while rclpy.ok() and not future.done():
        rclpy.spin_once(sim_node, timeout_sec=0.1)
        rclpy.spin_once(client_node, timeout_sec=0.1)
    return future.result()


def test_capture_and_score_returns_error_without_score(ros_nodes):
    sim_node, client_node = ros_nodes
    response = _call_service(client_node, sim_node)
    assert response.ok is False
    assert response.error_reason


def test_capture_and_score_returns_score_when_available(ros_nodes):
    sim_node, client_node = ros_nodes

    image = np.zeros((10, 10, 3), dtype=np.uint8)
    success, encoded = cv2.imencode(".jpg", image)
    assert success is True

    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = encoded.tobytes()

    sim_node._last_image_msg = msg
    sim_node.last_score = LightScore(score=0.5, is_good=False)

    response = _call_service(client_node, sim_node)
    assert response.ok is True
    assert response.score == pytest.approx(0.5, abs=1e-3)
    assert response.image_topic == sim_node._topics.images.output_capture_raw
