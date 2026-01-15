from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from curiosity_rosa_demo.infra.config_loader import load_all_configs
from curiosity_rosa_demo.infra.ros_io import (
    create_compressed_image_pub,
    create_compressed_image_sub,
    create_tf,
    lookup_x_in_world,
)
from curiosity_rosa_demo.sim.image_pipeline import apply_darkening_and_overlay
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
        self._last_image_msg: Optional[CompressedImage] = None

        self._image_pub = create_compressed_image_pub(
            self, self._topics.images.output_capture_compressed
        )
        self._image_sub = create_compressed_image_sub(
            self, self._topics.images.input_compressed, self._on_image
        )

        update_hz = float(self.get_parameter("score_update_hz").value)
        if update_hz <= 0.0:
            self.get_logger().warning(
                "score_update_hz must be positive; falling back to 5.0"
            )
            update_hz = 5.0
        period_sec = 1.0 / update_hz
        self._timer = self.create_timer(period_sec, self._tick)

    def _on_image(self, msg: CompressedImage) -> None:
        self._last_image_msg = msg

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
            self._process_image()
            return
        if not result.data or "x" not in result.data:
            self.get_logger().warning("TF lookup returned no x value")
            self._process_image()
            return

        x_value = float(result.data["x"])
        self.last_score = self._light_model.compute(x_value)
        self.last_pose_x = x_value
        self._process_image()

    def _process_image(self) -> None:
        if self._last_image_msg is None:
            return
        msg = self._last_image_msg
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("Failed to decode compressed image")
            return

        score = self.last_score.score if self.last_score else None
        is_good = self.last_score.is_good if self.last_score else None
        processed = apply_darkening_and_overlay(
            image,
            score=score,
            is_good=is_good,
            is_pending=self.last_score is None,
        )

        fmt = msg.format.lower() if msg.format else "jpeg"
        ext = ".png" if "png" in fmt else ".jpg"
        success, encoded = cv2.imencode(ext, processed)
        if not success:
            self.get_logger().warning("Failed to encode compressed image")
            return

        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = msg.format if msg.format else fmt
        out_msg.data = encoded.tobytes()
        self._image_pub.publish(out_msg)


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
