import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from curiosity_rosa_demo.sim.light_model import LightModel
from curiosity_rosa_demo.sim.simulator_node import SimulatorNode


def test_light_model_clamps_and_threshold():
    model = LightModel(x_min=0.0, x_good=5.0, score_threshold=0.8)

    assert model.compute(-1.0).score == 0.0
    assert model.compute(0.0).score == 0.0
    assert model.compute(5.0).score == 1.0
    assert model.compute(6.0).score == 1.0

    assert model.compute(3.9).is_good is False
    assert model.compute(4.1).is_good is True

    samples = [-1.0, 0.0, 1.0, 2.5, 5.0, 7.0]
    scores = [model.compute(value).score for value in samples]
    assert scores == sorted(scores)


@pytest.fixture
def ros_node():
    rclpy.init()
    config_dir = Path(__file__).resolve().parents[1] / "config"
    node = SimulatorNode(config_dir=config_dir)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_simulator_updates_last_score_from_tf(ros_node: SimulatorNode):
    node = ros_node
    node._world_frame = "map"
    node._base_frame = "base_link"
    broadcaster = StaticTransformBroadcaster(node)

    node._tick()
    assert node.last_score is None
    assert node.last_pose_x is None

    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = node._world_frame
    transform.child_frame_id = node._base_frame
    transform.transform.translation.x = 2.5
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    deadline = time.time() + 1.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._tf_buffer.can_transform(
            node._world_frame, node._base_frame, rclpy.time.Time()
        ):
            break

    node._tick()

    assert node.last_pose_x == pytest.approx(2.5, abs=1e-2)
    assert node.last_score is not None
    expected = node._light_model.compute(2.5)
    assert node.last_score.score == pytest.approx(expected.score, abs=1e-2)
