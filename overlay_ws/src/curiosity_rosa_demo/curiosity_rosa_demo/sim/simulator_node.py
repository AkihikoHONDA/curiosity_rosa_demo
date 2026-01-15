from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from curiosity_rosa_demo.infra.config_loader import load_all_configs
from curiosity_rosa_demo.infra.ros_io import create_tf, lookup_x_in_world
from curiosity_rosa_demo.sim.light_model import LightModel


class SimulatorNode(Node):
    def __init__(self, *, config_dir: Optional[Path] = None) -> None:
        super().__init__("simulator")
        self.declare_parameter("score_update_hz", 5.0)

        configs = load_all_configs(config_dir=config_dir, node=self)
        self._topics = configs.topics

        self._light_model = LightModel(
            x_min=configs.thresholds.light_model_x_min,
            x_good=configs.thresholds.light_model_x_good,
            score_threshold=configs.thresholds.quality_score_threshold,
        )

        self._tf_buffer, self._tf_listener = create_tf(self)
        self._world_frame = self._topics.tf.world_frame
        self._base_frame = self._topics.tf.base_frame

        self.last_score = None
        self.last_pose_x = None

        update_hz = float(self.get_parameter("score_update_hz").value)
        if update_hz <= 0.0:
            self.get_logger().warning(
                "score_update_hz must be positive; falling back to 5.0"
            )
            update_hz = 5.0
        period_sec = 1.0 / update_hz
        self._timer = self.create_timer(period_sec, self._tick)

    def _tick(self) -> None:
        result = lookup_x_in_world(
            self._tf_buffer,
            self._world_frame,
            self._base_frame,
            timeout_sec=0.2,
        )
        if not result.ok:
            self.get_logger().warning(
                f"TF lookup failed: {result.error_reason}"
            )
            return
        if not result.data or "x" not in result.data:
            self.get_logger().warning("TF lookup returned no x value")
            return

        x_value = float(result.data["x"])
        self.last_score = self._light_model.compute(x_value)
        self.last_pose_x = x_value


def main() -> None:
    rclpy.init()
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
