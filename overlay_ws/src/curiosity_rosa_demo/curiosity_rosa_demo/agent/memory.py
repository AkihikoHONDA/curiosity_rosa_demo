from __future__ import annotations

from collections import deque
from typing import Deque, Iterable, Optional


class ShortTermMemory:
    def __init__(self, enabled: bool, max_events: int) -> None:
        self.enabled = bool(enabled)
        self.max_events = int(max_events)
        self._events: Deque[str] = deque(maxlen=self.max_events)

    def add(self, entry: str) -> None:
        if not self.enabled:
            return
        self._events.append(str(entry))

    def recent(self) -> Iterable[str]:
        return list(self._events)

    def clear(self) -> None:
        self._events.clear()


def build_memory(enabled: bool, max_events: int) -> Optional[ShortTermMemory]:
    if not enabled:
        return None
    return ShortTermMemory(enabled=enabled, max_events=max_events)
