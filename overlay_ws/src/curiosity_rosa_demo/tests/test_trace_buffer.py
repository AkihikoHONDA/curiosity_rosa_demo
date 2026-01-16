from curiosity_rosa_demo.domain.models import TraceEvent
from curiosity_rosa_demo.trace.trace_buffer import TraceBuffer


def test_trace_buffer_latest_respects_maxlen():
    buffer = TraceBuffer(maxlen=2)
    first = TraceEvent(event_id="1", ts=1.0, kind="OBSERVE", message="one")
    second = TraceEvent(event_id="2", ts=2.0, kind="OBSERVE", message="two")
    third = TraceEvent(event_id="3", ts=3.0, kind="OBSERVE", message="three")

    buffer.append(first)
    buffer.append(second)
    buffer.append(third)

    latest = buffer.latest(2)
    assert [ev.event_id for ev in latest] == ["2", "3"]


def test_trace_buffer_as_lines_includes_fields():
    buffer = TraceBuffer(maxlen=3)
    event = TraceEvent(
        event_id="4",
        ts=4.0,
        kind="ACT",
        message="move",
        tool_name="move_forward",
        ok=True,
        score=0.12,
    )
    buffer.append(event)
    lines = buffer.as_lines(1)
    assert len(lines) == 1
    line = lines[0]
    assert "[ACT]" in line
    assert "move_forward" in line
    assert "ok=true" in line
    assert "score=0.12" in line
