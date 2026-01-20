from __future__ import annotations

from typing import Any, Optional

try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.text import Text
except Exception:  # pragma: no cover - optional dependency
    Console = None
    Panel = None
    Text = None


class RichStreamUI:
    def __init__(self) -> None:
        if Console is None or Panel is None or Text is None:
            raise RuntimeError("rich is unavailable")
        self._console = Console()

    def tool_start(self, name: str, tool_input: Any) -> None:
        content = self._format_payload(tool_input)
        body = Text.assemble((f"Tool Started: {name}\n", "bold"), (content, ""))
        panel = Panel(body, title=name, border_style="cyan")
        self._console.print()
        self._console.print(panel)
        self._console.print()

    def tool_end(
        self,
        name: str,
        ok: Optional[bool],
        error_reason: Optional[str],
        output: Any,
    ) -> None:
        content = self._format_payload(output)
        status = "unknown"
        if ok is True:
            status = "ok"
        elif ok is False:
            status = f"failed ({error_reason or 'unknown error'})"
        body = Text.assemble(
            (f"Tool Completed: {name}\n", "bold"),
            (f"Status: {status}\n", ""),
            (content, ""),
        )
        panel = Panel(body, title=name, border_style="green" if ok else "yellow")
        self._console.print()
        self._console.print(panel)
        self._console.print()

    def error(self, content: Any) -> None:
        body = Text.assemble(("Error\n", "bold"), (self._format_payload(content), ""))
        panel = Panel(body, border_style="red")
        self._console.print()
        self._console.print(panel)
        self._console.print()

    def final(self, content: str) -> None:
        body = Text.assemble(("LLM Final\n", "bold"), (content, ""))
        panel = Panel(body, border_style="magenta")
        self._console.print()
        self._console.print(panel)
        self._console.print()

    @staticmethod
    def _format_payload(payload: Any) -> str:
        if payload is None:
            return "(none)"
        if isinstance(payload, (str, int, float, bool)):
            return str(payload)
        try:
            import json

            return json.dumps(payload, ensure_ascii=False, indent=2)
        except TypeError:
            return repr(payload)
