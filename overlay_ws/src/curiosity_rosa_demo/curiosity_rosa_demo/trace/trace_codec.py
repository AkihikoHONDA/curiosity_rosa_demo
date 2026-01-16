from __future__ import annotations

import json
import time
import uuid
from typing import Any, Dict

from curiosity_rosa_demo.domain.models import TRACE_KINDS, TraceEvent


def encode_event(event: TraceEvent) -> str:
    data = event.to_dict()
    try:
        return json.dumps(data, ensure_ascii=False)
    except TypeError:
        safe = _make_json_safe(data)
        return json.dumps(safe, ensure_ascii=False)


def decode_event(payload: str) -> TraceEvent:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return _fallback_event("trace decode failed: invalid JSON", raw=payload)

    if not isinstance(data, dict):
        return _fallback_event("trace decode failed: root is not object", raw=data)

    kind = data.get("kind")
    if isinstance(kind, str) and kind not in TRACE_KINDS:
        return _fallback_event(f"trace decode failed: invalid kind '{kind}'", raw=data)

    try:
        return TraceEvent.from_dict(data)
    except Exception as exc:
        return _fallback_event(f"trace decode failed: {exc}", raw=data)


def _fallback_event(message: str, raw: Any) -> TraceEvent:
    return TraceEvent(
        event_id=str(uuid.uuid4()),
        ts=time.time(),
        kind="ERROR",
        message=message,
        data={"raw": _make_json_safe(raw)},
    )


def _make_json_safe(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list):
        return [_make_json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [_make_json_safe(item) for item in value]
    if isinstance(value, dict):
        safe: Dict[str, Any] = {}
        for key, item in value.items():
            safe[str(key)] = _make_json_safe(item)
        return safe
    return str(value)
