from __future__ import annotations

from collections import deque
from typing import Deque, List

from curiosity_rosa_demo.domain.models import TraceEvent


class TraceBuffer:
    def __init__(self, maxlen: int) -> None:
        if int(maxlen) <= 0:
            raise ValueError("maxlen must be positive")
        self._events: Deque[TraceEvent] = deque(maxlen=int(maxlen))

    def append(self, event: TraceEvent) -> None:
        self._events.append(event)

    def latest(self, n: int) -> List[TraceEvent]:
        if n <= 0:
            return []
        return list(self._events)[-n:]

    def as_lines(self, n: int) -> List[str]:
        lines: List[str] = []
        for event in self.latest(n):
            parts = [f"[{event.kind}]"]
            if event.tool_name:
                parts.append(event.tool_name)
            if event.ok is not None:
                parts.append(f"ok={'true' if event.ok else 'false'}")
            if event.score is not None:
                parts.append(f"score={event.score:.2f}")
            if event.message:
                parts.append(event.message)
            lines.append(" ".join(parts))
        return lines
