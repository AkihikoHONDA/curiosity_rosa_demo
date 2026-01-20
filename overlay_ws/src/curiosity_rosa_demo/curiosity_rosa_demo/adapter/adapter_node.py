from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

from curiosity_rosa_demo.infra.config_loader import load_all_configs
from curiosity_rosa_demo.infra.ros_io import create_tf, lookup_x_in_world


class AdapterNode(Node):
    def __init__(self, *, config_dir: Optional[Path] = None) -> None:
        super().__init__("adapter")
        configs = load_all_configs(config_dir=config_dir, node=self)
        self._topics = configs.topics

        self.last_error_reason: Optional[str] = None
        self._callback_group = ReentrantCallbackGroup()

        self._tf_buffer, self._tf_listener = create_tf(self)
        self._world_frame = self._topics.tf.world_frame
        self._base_frame = self._topics.tf.base_frame

        self._services = []
        self._services.append(
            self.create_service(
                Trigger,
                self._topics.adapter.mast_rotate.name,
                self._handle_mast_rotate,
                callback_group=self._callback_group,
            )
        )
        self._services.append(
            self.create_service(
                Trigger,
                self._topics.adapter.move_forward.name,
                self._handle_move_forward,
                callback_group=self._callback_group,
            )
        )
        self._services.append(
            self.create_service(
                Trigger,
                self._topics.adapter.turn_left.name,
                self._handle_turn_left,
                callback_group=self._callback_group,
            )
        )
        self._services.append(
            self.create_service(
                Trigger,
                self._topics.adapter.turn_right.name,
                self._handle_turn_right,
                callback_group=self._callback_group,
            )
        )
        self._services.append(
            self.create_service(
                Trigger,
                self._topics.adapter.move_stop.name,
                self._handle_move_stop,
                callback_group=self._callback_group,
            )
        )
        self._services.append(
            self.create_service(
                Trigger,
                self._topics.adapter.get_status.name,
                self._handle_get_status,
                callback_group=self._callback_group,
            )
        )

    def _handle_move_forward(self, _request, response):
        return self._handle_move(
            response,
            self._topics.curiosity.move_forward.name,
            enforce_exclusivity=False,
        )

    def _handle_turn_left(self, _request, response):
        return self._handle_move(
            response,
            self._topics.curiosity.turn_left.name,
            enforce_exclusivity=False,
        )

    def _handle_turn_right(self, _request, response):
        return self._handle_move(
            response,
            self._topics.curiosity.turn_right.name,
            enforce_exclusivity=False,
        )

    def _handle_move_stop(self, _request, response):
        return self._handle_move(
            response,
            self._topics.curiosity.move_stop.name,
            enforce_exclusivity=False,
        )

    def _handle_mast_rotate(self, _request, response):
        return self._call_curiosity(
            response,
            self._topics.curiosity.mast_rotate.name,
        )

    def _handle_get_status(self, _request, response):
        status = {
            "mast_is_open": None,
            "move_allowed": True,
            "last_error_reason": self.last_error_reason,
            "rover_x": self._lookup_rover_x(),
        }
        response.success = True
        response.message = json.dumps(status, ensure_ascii=True)
        return response

    def _lookup_rover_x(self) -> Optional[float]:
        result = lookup_x_in_world(
            self._tf_buffer,
            self._world_frame,
            self._base_frame,
            timeout_sec=0.1,
        )
        if not result.ok or not result.data or "x" not in result.data:
            return None
        return float(result.data["x"])

    def _handle_move(self, response, service_name: str, *, enforce_exclusivity: bool):
        return self._call_curiosity(response, service_name)

    def _call_curiosity(self, response, service_name: str, on_success=None):
        ok, error_reason = self._call_empty_service(service_name, timeout_sec=1.0)
        if ok:
            self.last_error_reason = None
            if on_success is not None:
                on_success()
            response.success = True
            response.message = ""
            return response
        self.last_error_reason = error_reason
        response.success = False
        response.message = error_reason or ""
        return response

    def _call_empty_service(self, service_name: str, timeout_sec: float):
        client = self.create_client(
            Empty, service_name, callback_group=self._callback_group
        )
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False, f"Service unavailable: {service_name}"
            future = client.call_async(Empty.Request())
            done = threading.Event()
            future.add_done_callback(lambda _future: done.set())
            if not done.wait(timeout=timeout_sec):
                return False, f"Service timeout: {service_name}"
            if future.exception() is not None:
                return False, f"Service exception: {future.exception()}"
            if future.result() is None:
                return False, "Service returned no response"
            return True, ""
        finally:
            self.destroy_client(client)

def main() -> None:
    rclpy.init()
    node = AdapterNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
