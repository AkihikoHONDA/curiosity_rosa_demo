from __future__ import annotations

from typing import Dict, Optional, Tuple


def parse_command(line: str) -> Tuple[str, str]:
    trimmed = line.strip()
    if not trimmed.startswith(":"):
        return "", trimmed
    parts = trimmed.split(maxsplit=1)
    command = parts[0][1:].strip().lower()
    args = parts[1].strip() if len(parts) > 1 else ""
    return command, args


def build_help_text(templates: Dict[str, Dict[str, str]]) -> str:
    lines = [
        "Commands:",
        "  :help       Show this help",
        "  :quit       Exit",
        "  :status     Call get_status tool (add 'llm' for explanation)",
        "  :cap        Call capture_and_score tool",
        "  :nudge      Call move_nudge tool",
        "  :mast_rotate Call mast_rotate tool",
        "  :demo       Run demo template",
        "",
        "Templates:",
    ]
    if not templates:
        lines.append("  (no templates configured)")
    else:
        for key, value in templates.items():
            label = value.get("label", "")
            text = value.get("text", "")
            if label:
                lines.append(f"  {key}: {label}")
            elif text:
                lines.append(f"  {key}: {text}")
            else:
                lines.append(f"  {key}")
    return "\n".join(lines)


def resolve_demo_text(
    templates: Dict[str, Dict[str, str]], template_key: str
) -> Optional[str]:
    entry = templates.get(template_key)
    if not entry:
        return None
    text = entry.get("text", "").strip()
    return text or None
