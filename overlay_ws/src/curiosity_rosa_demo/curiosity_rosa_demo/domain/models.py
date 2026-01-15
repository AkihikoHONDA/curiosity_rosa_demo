from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from .serde import time_from_dict, time_to_dict

TRACE_KINDS = {
    "OBSERVE",
    "HYPOTHESIZE",
    "DECIDE",
    "ACT",
    "RESULT",
    "ERROR",
}


@dataclass(frozen=True)
class RoverPose:
    x: float
    frame_id: str
    stamp: Any

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": float(self.x),
            "frame_id": self.frame_id,
            "stamp": time_to_dict(self.stamp),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RoverPose":
        return cls(
            x=float(data["x"]),
            frame_id=str(data["frame_id"]),
            stamp=time_from_dict(data.get("stamp")),
        )


@dataclass(frozen=True)
class MastState:
    is_open: bool
    yaw_rad: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"is_open": bool(self.is_open), "yaw_rad": self.yaw_rad}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MastState":
        yaw = data.get("yaw_rad")
        return cls(is_open=bool(data["is_open"]), yaw_rad=float(yaw) if yaw is not None else None)


@dataclass(frozen=True)
class LightScore:
    score: float
    is_good: bool

    def to_dict(self) -> Dict[str, Any]:
        return {"score": float(self.score), "is_good": bool(self.is_good)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "LightScore":
        return cls(score=float(data["score"]), is_good=bool(data["is_good"]))


@dataclass(frozen=True)
class CaptureResult:
    score: float
    is_good: bool
    image_topic: str
    stamp: Any

    def to_dict(self) -> Dict[str, Any]:
        return {
            "score": float(self.score),
            "is_good": bool(self.is_good),
            "image_topic": self.image_topic,
            "stamp": time_to_dict(self.stamp),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CaptureResult":
        return cls(
            score=float(data["score"]),
            is_good=bool(data["is_good"]),
            image_topic=str(data["image_topic"]),
            stamp=time_from_dict(data.get("stamp")),
        )


@dataclass(frozen=True)
class ToolResult:
    ok: bool
    error_reason: Optional[str] = None
    data: Optional[Dict[str, Any]] = None

    def __post_init__(self) -> None:
        if not self.ok:
            if self.error_reason is None or not str(self.error_reason).strip():
                raise ValueError("ToolResult.error_reason is required when ok=False")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": bool(self.ok),
            "error_reason": self.error_reason,
            "data": self.data,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ToolResult":
        return cls(
            ok=bool(data["ok"]),
            error_reason=data.get("error_reason"),
            data=data.get("data"),
        )


@dataclass(frozen=True)
class TraceEvent:
    event_id: str
    ts: float
    kind: str
    message: str
    tool_name: Optional[str] = None
    ok: Optional[bool] = None
    error_reason: Optional[str] = None
    score: Optional[float] = None
    data: Optional[Dict[str, Any]] = None

    def __post_init__(self) -> None:
        if self.kind not in TRACE_KINDS:
            raise ValueError(f"TraceEvent.kind must be one of {sorted(TRACE_KINDS)}")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "event_id": self.event_id,
            "ts": float(self.ts),
            "kind": self.kind,
            "message": self.message,
            "tool_name": self.tool_name,
            "ok": self.ok,
            "error_reason": self.error_reason,
            "score": float(self.score) if self.score is not None else None,
            "data": self.data,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TraceEvent":
        return cls(
            event_id=str(data["event_id"]),
            ts=float(data["ts"]),
            kind=str(data["kind"]),
            message=str(data["message"]),
            tool_name=data.get("tool_name"),
            ok=data.get("ok"),
            error_reason=data.get("error_reason"),
            score=float(data["score"]) if data.get("score") is not None else None,
            data=data.get("data"),
        )
