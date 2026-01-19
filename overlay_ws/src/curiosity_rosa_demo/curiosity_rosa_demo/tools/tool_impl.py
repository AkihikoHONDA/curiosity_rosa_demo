from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy

from curiosity_rosa_demo.domain.models import ToolResult
from curiosity_rosa_demo.infra.config_loader import load_all_configs
from curiosity_rosa_demo.infra.ros_io import call_trigger_service

try:
    from curiosity_rosa_demo.srv import CaptureAndScore
except Exception:  # pragma: no cover - available after rosidl build
    CaptureAndScore = None


class ToolContext:
    def __init__(self, node: Any, *, config_dir: Optional[Path] = None) -> None:
        self.node = node
        configs = load_all_configs(config_dir=config_dir, node=node)
        self.topics = configs.topics
        self.tool_costs = configs.tool_costs.tools
        self.thresholds = configs.thresholds

    def cost_for(self, tool_name: str) -> int:
        return int(self.tool_costs.get(tool_name, 0))


def _with_cost(result: ToolResult, cost: int) -> ToolResult:
    data: Dict[str, Any]
    if result.data is None:
        data = {"cost": cost}
    else:
        data = dict(result.data)
        data["cost"] = cost
    return ToolResult(ok=result.ok, error_reason=result.error_reason, data=data)


def _call_trigger_tool(ctx: ToolContext, tool_name: str, service_name: str) -> ToolResult:
    result = call_trigger_service(ctx.node, service_name, timeout_sec=1.0)
    return _with_cost(result, ctx.cost_for(tool_name))


def capture_and_score(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    if CaptureAndScore is None:
        return ToolResult(ok=False, error_reason="CaptureAndScore type unavailable")

    cost = ctx.cost_for("capture_and_score")
    client = node.create_client(
        CaptureAndScore, ctx.topics.services.capture_and_score.name
    )
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            return ToolResult(
                ok=False,
                error_reason="Service unavailable: /capture_and_score",
                data={"score": 0.0, "is_good": False, "cost": cost},
            )
        future = client.call_async(CaptureAndScore.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if not future.done():
            return ToolResult(
                ok=False,
                error_reason="Service timeout: /capture_and_score",
                data={"score": 0.0, "is_good": False, "cost": cost},
            )
        if future.exception() is not None:
            return ToolResult(
                ok=False,
                error_reason=f"Service exception: {future.exception()}",
                data={"score": 0.0, "is_good": False, "cost": cost},
            )
        response = future.result()
        if response is None:
            return ToolResult(
                ok=False,
                error_reason="Service returned no response",
                data={"score": 0.0, "is_good": False, "cost": cost},
            )
        score = float(response.score) if response.ok else 0.0
        is_good = bool(response.is_good) if response.ok else False
        data = {
            "score": score,
            "is_good": is_good,
            "image_topic": response.image_topic,
            "stamp": response.stamp,
            "debug": response.debug,
            "cost": cost,
        }
        if response.ok:
            return ToolResult(ok=True, data=data)
        return ToolResult(
            ok=False,
            error_reason=response.error_reason or "capture failed",
            data=data,
        )
    finally:
        node.destroy_client(client)


def mast_open(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "mast_open", ctx.topics.adapter.mast_open.name)


def mast_close(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "mast_close", ctx.topics.adapter.mast_close.name)


def mast_rotate(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "mast_rotate", ctx.topics.adapter.mast_rotate.name)


def move_nudge(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    cost = ctx.cost_for("move_nudge")
    forward = call_trigger_service(
        ctx.node, ctx.topics.adapter.move_forward.name, timeout_sec=1.0
    )
    if not forward.ok:
        return _with_cost(forward, cost)
    duration = float(ctx.thresholds.move_nudge_duration_sec)
    time.sleep(duration)
    stop = call_trigger_service(
        ctx.node, ctx.topics.adapter.move_stop.name, timeout_sec=1.0
    )
    if not stop.ok:
        return _with_cost(stop, cost)
    return ToolResult(ok=True, data={"duration_sec": duration, "cost": cost})


def move_forward(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "move_forward", ctx.topics.adapter.move_forward.name)


def turn_left(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "turn_left", ctx.topics.adapter.turn_left.name)


def turn_right(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "turn_right", ctx.topics.adapter.turn_right.name)


def move_stop(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    return _call_trigger_tool(ctx, "move_stop", ctx.topics.adapter.move_stop.name)


def get_status(node: Any, *, config_dir: Optional[Path] = None) -> ToolResult:
    ctx = ToolContext(node, config_dir=config_dir)
    result = call_trigger_service(
        ctx.node, ctx.topics.adapter.get_status.name, timeout_sec=1.0
    )
    cost = ctx.cost_for("get_status")
    if not result.ok:
        return _with_cost(result, cost)
    message = ""
    if result.data and "message" in result.data:
        message = str(result.data["message"])
    if not message:
        return ToolResult(ok=True, data={"cost": cost})
    try:
        decoded = json.loads(message)
        if not isinstance(decoded, dict):
            decoded = {"value": decoded}
        decoded["cost"] = cost
        return ToolResult(ok=True, data=decoded)
    except json.JSONDecodeError:
        return ToolResult(ok=True, data={"raw_message": message, "cost": cost})
