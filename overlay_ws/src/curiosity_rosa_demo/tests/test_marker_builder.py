from visualization_msgs.msg import Marker

from curiosity_rosa_demo.viz.marker_builder import build_marker_array


def test_marker_builder_outputs_markers():
    markers = build_marker_array(
        frame_id="odom",
        stamp=None,
        bright_zone=(0.0, 5.0),
        lines=["[OBSERVE] capture ok=true"],
        score_points=85,
        origin=(1.0, 2.0, 3.0),
    )
    assert len(markers.markers) >= 2
    marker_types = {marker.type for marker in markers.markers}
    assert Marker.CUBE in marker_types
    assert Marker.TEXT_VIEW_FACING in marker_types
