import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from curiosity_rosa_demo.infra.ros_io import (
    call_empty_service,
    call_trigger_service,
    create_tf,
    lookup_x_in_world,
)


@pytest.fixture
def ros_context():
    rclpy.init()
    node = Node("test_ros_io")
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_call_empty_service_success(ros_context: Node):
    node = ros_context

    def _handle(_request, _response):
        return Empty.Response()

    node.create_service(Empty, "/test_empty", _handle)
    result = call_empty_service(node, "/test_empty", timeout_sec=1.0)
    assert result.ok is True


def test_call_trigger_service_success_and_failure(ros_context: Node):
    node = ros_context

    def _handle_success(_request, _response):
        return Trigger.Response(success=True, message="ok")

    def _handle_failure(_request, _response):
        return Trigger.Response(success=False, message="nope")

    node.create_service(Trigger, "/test_trigger_ok", _handle_success)
    node.create_service(Trigger, "/test_trigger_bad", _handle_failure)

    ok_result = call_trigger_service(node, "/test_trigger_ok", timeout_sec=1.0)
    assert ok_result.ok is True

    bad_result = call_trigger_service(node, "/test_trigger_bad", timeout_sec=1.0)
    assert bad_result.ok is False
    assert bad_result.error_reason == "nope"


def test_call_service_timeout_returns_error(ros_context: Node):
    result = call_empty_service(ros_context, "/no_such_service", timeout_sec=0.2)
    assert result.ok is False
    assert result.error_reason


def test_lookup_x_in_world_success(ros_context: Node):
    node = ros_context
    tf_buffer, _listener = create_tf(node)
    broadcaster = StaticTransformBroadcaster(node)

    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = "map"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = 1.23
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    deadline = time.time() + 1.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if tf_buffer.can_transform("map", "base_link", rclpy.time.Time()):
            break

    result = lookup_x_in_world(tf_buffer, "map", "base_link", timeout_sec=1.0)
    assert result.ok is True
    assert result.data is not None
    assert result.data["x"] == pytest.approx(1.23, abs=1e-2)
