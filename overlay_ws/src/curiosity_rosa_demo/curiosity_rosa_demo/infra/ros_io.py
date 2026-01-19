from __future__ import annotations

from typing import Any, Callable, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import CompressedImage, Image

from curiosity_rosa_demo.domain.models import ToolResult


DEFAULT_TRIGGER_FAILURE = "Trigger service failed"


def wait_for_service(node: Any, service_name: str, timeout_sec: float) -> bool:
    client = node.create_client(Empty, service_name)
    try:
        return bool(client.wait_for_service(timeout_sec=timeout_sec))
    finally:
        node.destroy_client(client)


def call_empty_service(
    node: Any, service_name: str, timeout_sec: float
) -> ToolResult:
    client = node.create_client(Empty, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return ToolResult(
                ok=False,
                error_reason=f"Service unavailable: {service_name}",
            )
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            return ToolResult(ok=False, error_reason=f"Service timeout: {service_name}")
        if future.exception() is not None:
            return ToolResult(
                ok=False,
                error_reason=f"Service exception: {future.exception()}",
            )
        return ToolResult(ok=True)
    except Exception as exc:
        return ToolResult(ok=False, error_reason=f"Service error: {exc}")
    finally:
        node.destroy_client(client)


def call_trigger_service(
    node: Any, service_name: str, timeout_sec: float
) -> ToolResult:
    client = node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return ToolResult(
                ok=False,
                error_reason=f"Service unavailable: {service_name}",
            )
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            return ToolResult(ok=False, error_reason=f"Service timeout: {service_name}")
        if future.exception() is not None:
            return ToolResult(
                ok=False,
                error_reason=f"Service exception: {future.exception()}",
            )
        response = future.result()
        if response is None:
            return ToolResult(ok=False, error_reason="Service returned no response")
        if response.success:
            data = {"message": response.message} if response.message else {}
            return ToolResult(ok=True, data=data or None)
        message = response.message.strip() if response.message else ""
        return ToolResult(ok=False, error_reason=message or DEFAULT_TRIGGER_FAILURE)
    except Exception as exc:
        return ToolResult(ok=False, error_reason=f"Service error: {exc}")
    finally:
        node.destroy_client(client)


def create_compressed_image_pub(
    node: Any, topic: str, qos: QoSProfile | None = None
):
    qos_profile = qos or qos_profile_sensor_data
    return node.create_publisher(CompressedImage, topic, qos_profile)


def create_image_pub(
    node: Any,
    topic: str,
    qos: QoSProfile | None = None,
    *,
    reliable: bool = False,
):
    if qos is not None:
        qos_profile = qos
    else:
        qos_profile = qos_profile_sensor_data
        if reliable:
            qos_profile = QoSProfile(
                depth=qos_profile.depth,
                reliability=ReliabilityPolicy.RELIABLE,
            )
    return node.create_publisher(Image, topic, qos_profile)


def create_compressed_image_sub(
    node: Any,
    topic: str,
    callback: Callable[[CompressedImage], None],
    qos: QoSProfile | None = None,
):
    qos_profile = qos or qos_profile_sensor_data
    return node.create_subscription(CompressedImage, topic, callback, qos_profile)


def create_image_sub(
    node: Any,
    topic: str,
    callback: Callable[[Image], None],
    qos: QoSProfile | None = None,
):
    qos_profile = qos or qos_profile_sensor_data
    return node.create_subscription(Image, topic, callback, qos_profile)


def create_trace_pub(node: Any, topic: str, qos: QoSProfile | None = None):
    qos_profile = qos or QoSProfile(depth=10)
    return node.create_publisher(String, topic, qos_profile)


def create_tf(node: Any) -> Tuple[Buffer, TransformListener]:
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    return tf_buffer, tf_listener


def lookup_x_in_world(
    tf_buffer: Buffer,
    world_frame: str,
    base_frame: str,
    timeout_sec: float,
) -> ToolResult:
    try:
        transform = tf_buffer.lookup_transform(
            world_frame,
            base_frame,
            Time(),
            timeout=Duration(seconds=timeout_sec),
        )
        x_value = float(transform.transform.translation.x)
        return ToolResult(ok=True, data={"x": x_value})
    except TransformException as exc:
        return ToolResult(ok=False, error_reason=str(exc))
    except Exception as exc:
        return ToolResult(ok=False, error_reason=f"TF error: {exc}")
