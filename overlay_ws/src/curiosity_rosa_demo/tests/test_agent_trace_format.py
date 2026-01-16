from curiosity_rosa_demo.agent.agent_node import build_tool_trace_event
from curiosity_rosa_demo.domain.models import ToolResult
from curiosity_rosa_demo.trace.trace_codec import encode_event


def test_tool_result_to_trace_event_json():
    result = ToolResult(ok=True, data={"score": 0.8, "cost": 1})
    event = build_tool_trace_event("capture_and_score", result)
    payload = encode_event(event)
    assert '"kind"' in payload
    assert '"RESULT"' in payload
    assert '"capture_and_score"' in payload
