import json
import threading
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

from curiosity_rosa_demo.adapter.adapter_node import AdapterNode, EXCLUSIVITY_ERROR


@pytest.fixture
def ros_nodes():
    rclpy.init()
    config_dir = Path(__file__).resolve().parents[1] / "config"
    adapter_node = AdapterNode(config_dir=config_dir)
    client_node = Node("test_adapter_client")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(adapter_node)
    executor.add_node(client_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    counters = {
        "move_forward": 0,
        "turn_left": 0,
        "turn_right": 0,
        "move_stop": 0,
        "mast_open": 0,
        "mast_close": 0,
        "mast_rotate": 0,
    }

    def _make_handler(key):
        def _handle(_request, _response):
            counters[key] += 1
            return Empty.Response()
        return _handle

    # Curiosity dummy services
    adapter_node.create_service(
        Empty, "/move_forward", _make_handler("move_forward"),
        callback_group=adapter_node._callback_group,
    )
    adapter_node.create_service(
        Empty, "/turn_left", _make_handler("turn_left"),
        callback_group=adapter_node._callback_group,
    )
    adapter_node.create_service(
        Empty, "/turn_right", _make_handler("turn_right"),
        callback_group=adapter_node._callback_group,
    )
    adapter_node.create_service(
        Empty, "/move_stop", _make_handler("move_stop"),
        callback_group=adapter_node._callback_group,
    )
    adapter_node.create_service(
        Empty, "/mast_open", _make_handler("mast_open"),
        callback_group=adapter_node._callback_group,
    )
    adapter_node.create_service(
        Empty, "/mast_close", _make_handler("mast_close"),
        callback_group=adapter_node._callback_group,
    )
    adapter_node.create_service(
        Empty, "/mast_rotate", _make_handler("mast_rotate"),
        callback_group=adapter_node._callback_group,
    )

    try:
        yield adapter_node, client_node, counters
    finally:
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        adapter_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def _call_trigger(client_node: Node, service_name: str):
    client = client_node.create_client(Trigger, service_name)
    assert client.wait_for_service(timeout_sec=1.0)
    future = client.call_async(Trigger.Request())
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    assert done.wait(timeout=2.0)
    return future.result()


def test_exclusivity_and_status(ros_nodes):
    adapter_node, client_node, counters = ros_nodes

    # mast_open succeeds and sets mast_is_open
    response = _call_trigger(client_node, "/adapter/mast_open")
    assert response.success is True
    assert adapter_node.mast_is_open is True
    assert counters["mast_open"] == 1

    # move_forward is rejected while mast open
    response = _call_trigger(client_node, "/adapter/move_forward")
    assert response.success is False
    assert response.message == EXCLUSIVITY_ERROR
    assert counters["move_forward"] == 0

    # move_stop is allowed even while mast open
    response = _call_trigger(client_node, "/adapter/move_stop")
    assert response.success is True
    assert counters["move_stop"] == 1

    # get_status returns JSON with expected keys
    response = _call_trigger(client_node, "/adapter/get_status")
    assert response.success is True
    status = json.loads(response.message)
    assert status["mast_is_open"] is True
    assert status["move_allowed"] is False
    assert status["last_error_reason"] is None
    assert "rover_x" in status

    # mast_close allows moves again
    response = _call_trigger(client_node, "/adapter/mast_close")
    assert response.success is True
    assert adapter_node.mast_is_open is False

    response = _call_trigger(client_node, "/adapter/move_forward")
    assert response.success is True
    assert counters["move_forward"] == 1
