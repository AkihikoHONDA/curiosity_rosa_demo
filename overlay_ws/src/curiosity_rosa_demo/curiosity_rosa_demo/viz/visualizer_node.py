from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from curiosity_rosa_demo.domain.models import TraceEvent
from curiosity_rosa_demo.infra.config_loader import load_all_configs
from curiosity_rosa_demo.trace.trace_buffer import TraceBuffer
from curiosity_rosa_demo.infra.ros_io import create_tf, lookup_x_in_world
from curiosity_rosa_demo.trace.trace_codec import decode_event
from curiosity_rosa_demo.viz.marker_builder import build_marker_array


class VisualizerNode(Node):
    def __init__(self, *, config_dir: Optional[Path] = None) -> None:
        super().__init__("visualizer")
        configs = load_all_configs(config_dir=config_dir, node=self)
        self._topics = configs.topics
        self._thresholds = configs.thresholds
        self._tf_buffer, self._tf_listener = create_tf(self)

        buffer_size = self._thresholds.trace_buffer_size or 30
        self._trace_buffer = TraceBuffer(buffer_size)
        self._latest_score_points: Optional[int] = None

        self._trace_sub = self.create_subscription(
            String, self._topics.trace.events, self._on_trace, 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, self._topics.viz.marker_array, 10
        )

        self._timer = self.create_timer(0.5, self._publish_markers)

    def _on_trace(self, msg: String) -> None:
        event = decode_event(msg.data)
        self._trace_buffer.append(event)
        self._update_score(event)

    def _update_score(self, event: TraceEvent) -> None:
        if event.tool_name != "capture_and_score":
            return
        if event.score is None:
            return
        points = int(round(float(event.score) * 100.0))
        points = max(0, min(100, points))
        self._latest_score_points = points

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        bright_min = self._thresholds.viz_bright_zone_x_min
        bright_max = self._thresholds.viz_bright_zone_x_max
        if bright_min is None or bright_max is None:
            bright_min, bright_max = (
                self._thresholds.light_model_x_good,
                self._thresholds.light_model_x_good + 5.0,
            )
        lines = self._trace_buffer.as_lines(self._trace_buffer._events.maxlen)
        origin = self._get_rover_origin()
        markers = build_marker_array(
            frame_id=self._topics.tf.world_frame,
            stamp=now,
            bright_zone=(float(bright_min), float(bright_max)),
            lines=lines,
            score_points=self._latest_score_points,
            origin=origin,
        )
        self._marker_pub.publish(markers)

    def _get_rover_origin(self) -> tuple[float, float, float]:
        result = lookup_x_in_world(
            self._tf_buffer,
            self._topics.tf.world_frame,
            "chassis",
            timeout_sec=0.1,
        )
        if not result.ok or not result.data or "x" not in result.data:
            return (0.0, 0.0, 0.0)
        return (float(result.data["x"]), 0.0, 0.0)


def main() -> None:
    rclpy.init()
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
