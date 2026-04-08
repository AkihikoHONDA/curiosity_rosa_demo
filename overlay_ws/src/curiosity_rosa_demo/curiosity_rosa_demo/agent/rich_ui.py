from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

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
        self._step = 0
        self._pending_steps: Dict[str, List[int]] = {}

    def tool_start(self, name: str, tool_input: Any) -> None:
        self._step += 1
        step = self._step
        self._pending_steps.setdefault(name, []).append(step)

        compact_input = self._compact_input(tool_input)
        if compact_input:
            self._console.print(
                f"[cyan]Step {step:02d} START[/cyan] {name}  input={compact_input}"
            )
        else:
            self._console.print(f"[cyan]Step {step:02d} START[/cyan] {name}")

    def tool_end(
        self,
        name: str,
        ok: Optional[bool],
        error_reason: Optional[str],
        output: Any,
    ) -> None:
        step = self._pop_step(name)
        status = "unknown"
        if ok is True:
            status = "ok"
        elif ok is False:
            status = f"failed ({error_reason or 'unknown error'})"

        summary, key_metrics = self._summarize_output(output)
        body = Text()
        body.append(f"Step {step:02d} RESULT\n", style="bold")
        body.append(f"Action: {name}\n")
        body.append(f"Status: {status}\n")
        body.append(f"Summary: {summary}")
        if key_metrics:
            body.append("\n")
            body.append("Key Metrics: ", style="bold magenta")
            body.append(key_metrics, style="bold magenta")

        panel = Panel(body, title="Tool", border_style="green" if ok else "yellow")
        self._console.print(panel)

    def error(self, content: Any) -> None:
        body = Text.assemble(("Error\n", "bold"), (self._format_payload(content), ""))
        panel = Panel(body, title="Error", border_style="red")
        self._console.print(panel)

    def final(self, content: str) -> None:
        panel = Panel(content, title="LLM Final", border_style="magenta")
        self._console.print(panel)

    def _pop_step(self, name: str) -> int:
        steps = self._pending_steps.get(name)
        if not steps:
            return 0
        step = steps.pop(0)
        if not steps:
            self._pending_steps.pop(name, None)
        return step

    def _compact_input(self, payload: Any) -> str:
        if payload in (None, {}, ""):
            return ""
        return self._format_payload(payload)

    def _summarize_output(self, output: Any) -> Tuple[str, str]:
        ok, reason, data = self._extract_output_fields(output)
        parts: List[str] = []
        key_metrics: List[str] = []

        if ok is True:
            parts.append("ok=true")
        elif ok is False:
            parts.append("ok=false")
        if reason:
            parts.append(f"reason={reason}")

        if isinstance(data, dict):
            if "score" in data:
                try:
                    score_text = f"{float(data['score']):.4f}"
                except Exception:
                    score_text = str(data["score"])
                parts.append(f"score={score_text}")
                key_metrics.append(f"score={score_text}")
            if "is_good" in data:
                is_good_text = str(data["is_good"])
                parts.append(f"is_good={is_good_text}")
                key_metrics.append(f"is_good={is_good_text}")
            if "duration_sec" in data:
                try:
                    parts.append(f"duration={float(data['duration_sec']):.1f}s")
                except Exception:
                    parts.append(f"duration={data['duration_sec']}")
            if "rover_x" in data:
                try:
                    parts.append(f"rover_x={float(data['rover_x']):.3f}")
                except Exception:
                    parts.append(f"rover_x={data['rover_x']}")
            if "cost" in data:
                parts.append(f"cost={data['cost']}")
            if "last_error_reason" in data and data["last_error_reason"]:
                parts.append(f"last_error={data['last_error_reason']}")

        if not parts:
            return self._format_payload(output), ""
        return ", ".join(parts), ", ".join(key_metrics)

    @staticmethod
    def _extract_output_fields(output: Any) -> Tuple[Optional[bool], Optional[str], Optional[dict]]:
        if output is None:
            return None, None, None

        if hasattr(output, "ok") and hasattr(output, "error_reason") and hasattr(output, "data"):
            ok = getattr(output, "ok")
            reason = getattr(output, "error_reason")
            data = getattr(output, "data")
            return (
                ok if isinstance(ok, bool) else None,
                reason if isinstance(reason, str) and reason else None,
                data if isinstance(data, dict) else None,
            )

        if isinstance(output, dict):
            ok = output.get("ok")
            reason = output.get("error_reason")
            data = output.get("data")
            return (
                ok if isinstance(ok, bool) else None,
                reason if isinstance(reason, str) and reason else None,
                data if isinstance(data, dict) else (output if "data" not in output else None),
            )

        return None, None, None

    @staticmethod
    def _format_payload(payload: Any) -> str:
        if payload is None:
            return "(none)"
        if isinstance(payload, (str, int, float, bool)):
            return str(payload)
        try:
            import json

            return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        except TypeError:
            return repr(payload)
