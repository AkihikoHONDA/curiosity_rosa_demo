import json

from curiosity_rosa_demo.domain.models import TraceEvent
from curiosity_rosa_demo.trace.trace_codec import decode_event, encode_event


def test_encode_decode_roundtrip():
    event = TraceEvent(
        event_id="evt-1",
        ts=123.4,
        kind="RESULT",
        message="capture done",
        tool_name="capture_and_score",
        ok=True,
        score=0.42,
        data={"image_topic": "/capture/image_raw"},
    )
    encoded = encode_event(event)
    decoded = decode_event(encoded)
    assert decoded.event_id == event.event_id
    assert decoded.kind == event.kind
    assert decoded.message == event.message
    assert decoded.tool_name == event.tool_name
    assert decoded.ok == event.ok
    assert decoded.score == event.score
    assert decoded.data == event.data


def test_decode_invalid_json_returns_error():
    decoded = decode_event("{bad json")
    assert decoded.kind == "ERROR"
    assert "invalid JSON" in decoded.message


def test_decode_invalid_kind_returns_error():
    payload = json.dumps(
        {"event_id": "evt-2", "ts": 0.0, "kind": "BAD", "message": "nope"}
    )
    decoded = decode_event(payload)
    assert decoded.kind == "ERROR"
    assert "invalid kind" in decoded.message
