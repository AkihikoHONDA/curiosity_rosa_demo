from __future__ import annotations

from typing import List, Optional, Tuple

from visualization_msgs.msg import Marker, MarkerArray


def build_marker_array(
    *,
    frame_id: str,
    stamp,
    bright_zone: Tuple[float, float],
    lines: List[str],
    score_points: Optional[int],
    origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MarkerArray:
    markers: List[Marker] = []
    markers.append(_build_bright_zone(frame_id, stamp, bright_zone))
    markers.extend(_build_log_markers(frame_id, stamp, lines, origin))
    if score_points is not None:
        markers.append(_build_score_marker(frame_id, stamp, score_points, origin))
    return MarkerArray(markers=markers)


def _build_bright_zone(
    frame_id: str, stamp, bright_zone: Tuple[float, float]
) -> Marker:
    x_min, x_max = bright_zone
    marker = _base_marker(
        frame_id=frame_id,
        stamp=stamp,
        marker_id=1,
        marker_type=Marker.CUBE,
    )
    marker.pose.position.x = float(x_min + x_max) * 0.5
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.1
    marker.scale.x = float(x_max - x_min)
    marker.scale.y = 2.0
    marker.scale.z = 0.02
    marker.color.r = 0.2
    marker.color.g = 0.8
    marker.color.b = 0.2
    marker.color.a = 0.35
    return marker


def _build_log_markers(
    frame_id: str, stamp, lines: List[str], origin: Tuple[float, float, float]
) -> List[Marker]:
    markers: List[Marker] = []
    base_x = origin[0]
    base_y = origin[1] - 2.0
    base_z = origin[2] + 0.6
    for idx, line in enumerate(lines):
        marker = _base_marker(
            frame_id=frame_id,
            stamp=stamp,
            marker_id=10 + idx,
            marker_type=Marker.TEXT_VIEW_FACING,
        )
        marker.text = line
        marker.pose.position.x = base_x
        marker.pose.position.y = base_y
        marker.pose.position.z = base_z + 0.2 * idx
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        markers.append(marker)
    return markers


def _build_score_marker(
    frame_id: str, stamp, score_points: int, origin: Tuple[float, float, float]
) -> Marker:
    marker = _base_marker(
        frame_id=frame_id,
        stamp=stamp,
        marker_id=2,
        marker_type=Marker.TEXT_VIEW_FACING,
    )
    marker.text = f"Score: {score_points}/100"
    marker.pose.position.x = origin[0]
    marker.pose.position.y = origin[1] + 2.0
    marker.pose.position.z = origin[2] + 0.8
    marker.scale.z = 0.4
    marker.color.r = 1.0
    marker.color.g = 0.9
    marker.color.b = 0.2
    marker.color.a = 0.95
    return marker


def _base_marker(
    *,
    frame_id: str,
    stamp,
    marker_id: int,
    marker_type: int,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "curiosity_viz"
    marker.id = int(marker_id)
    marker.type = int(marker_type)
    marker.action = Marker.ADD
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    return marker
