import pytest

from curiosity_rosa_demo.domain.models import CaptureResult, ToolResult, TraceEvent


def _normalize_stamp(value):
    if value is None:
        return None
    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        return {"sec": int(value.sec), "nanosec": int(value.nanosec)}
    return value


def test_capture_result_roundtrip():
    stamp = {"sec": 12, "nanosec": 34}
    original = CaptureResult(
        score=0.75,
        is_good=False,
        image_topic="/capture/image_raw",
        stamp=stamp,
    )
    data = original.to_dict()
    restored = CaptureResult.from_dict(data)
    assert restored.score == pytest.approx(original.score)
    assert restored.is_good == original.is_good
    assert restored.image_topic == original.image_topic
    assert _normalize_stamp(restored.stamp) == stamp


def test_tool_result_roundtrip():
    original = ToolResult(ok=True, error_reason=None, data={"score": 0.9})
    data = original.to_dict()
    restored = ToolResult.from_dict(data)
    assert restored.ok is True
    assert restored.error_reason is None
    assert restored.data == {"score": 0.9}


def test_tool_result_error_reason_required():
    with pytest.raises(ValueError):
        ToolResult(ok=False, error_reason="")


def test_trace_event_roundtrip():
    original = TraceEvent(
        event_id="evt-1",
        ts=123.4,
        kind="OBSERVE",
        message="captured",
        tool_name="capture_and_score",
        ok=True,
        score=0.42,
        data={"image_topic": "/capture/image_raw"},
    )
    data = original.to_dict()
    restored = TraceEvent.from_dict(data)
    assert restored.event_id == original.event_id
    assert restored.kind == original.kind
    assert restored.message == original.message
    assert restored.tool_name == original.tool_name
    assert restored.ok == original.ok
    assert restored.score == pytest.approx(original.score)
    assert restored.data == original.data


def test_trace_event_invalid_kind():
    with pytest.raises(ValueError):
        TraceEvent(event_id="evt-bad", ts=0.0, kind="BAD", message="nope")
