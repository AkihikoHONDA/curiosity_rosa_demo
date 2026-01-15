from __future__ import annotations

from typing import Any, Dict, Optional


def time_to_dict(value: Any) -> Optional[Dict[str, int]]:
    if value is None:
        return None
    if _has_time_fields(value):
        return {"sec": int(value.sec), "nanosec": int(value.nanosec)}
    if isinstance(value, dict) and "sec" in value and "nanosec" in value:
        return {"sec": int(value["sec"]), "nanosec": int(value["nanosec"])}
    raise ValueError("stamp must be a Time-like object or {sec, nanosec} dict")


def time_from_dict(value: Any) -> Any:
    if value is None:
        return None
    if _has_time_fields(value):
        return value
    if isinstance(value, dict) and "sec" in value and "nanosec" in value:
        try:
            from builtin_interfaces.msg import Time
        except Exception:
            return {"sec": int(value["sec"]), "nanosec": int(value["nanosec"])}
        return Time(sec=int(value["sec"]), nanosec=int(value["nanosec"]))
    raise ValueError("stamp must be a Time-like object or {sec, nanosec} dict")


def _has_time_fields(value: Any) -> bool:
    return hasattr(value, "sec") and hasattr(value, "nanosec")
