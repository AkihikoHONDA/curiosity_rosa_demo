from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from curiosity_rosa_demo.agent.console import (
    build_help_text,
    parse_command,
    resolve_demo_text,
)
from curiosity_rosa_demo.agent.rosa_agent_factory import RosaAgentFactory
from curiosity_rosa_demo.domain.models import ToolResult, TraceEvent
from curiosity_rosa_demo.infra.config_loader import load_all_configs
from curiosity_rosa_demo.infra.ros_io import create_trace_pub
from curiosity_rosa_demo.tools import tool_impl
from curiosity_rosa_demo.trace.trace_buffer import TraceBuffer
from curiosity_rosa_demo.trace.trace_codec import encode_event


DEMO_TEMPLATE_KEY = "rover_status"


def build_tool_trace_event(tool_name: str, result: ToolResult) -> TraceEvent:
    event_id = str(uuid.uuid4())
    ts = time.time()
    if result.ok:
        kind = "RESULT"
        message = f"{tool_name} ok"
    else:
        kind = "ERROR"
        reason = result.error_reason or "unknown error"
        message = f"{tool_name} failed: {reason}"
    score = None
    if result.data and "score" in result.data:
        try:
            score = float(result.data["score"])
        except (TypeError, ValueError):
            score = None
    return TraceEvent(
        event_id=event_id,
        ts=ts,
        kind=kind,
        message=message,
        tool_name=tool_name,
        ok=result.ok,
        error_reason=result.error_reason,
        score=score,
        data=result.data,
    )


def extract_llm_text(response: Any) -> str:
    if isinstance(response, str):
        return response
    if isinstance(response, dict):
        for key in ("output", "text", "content", "response", "result"):
            value = response.get(key)
            if isinstance(value, str) and value.strip():
                return value
    return str(response)


def summarize_tool_result(tool_name: str, result: ToolResult) -> str:
    if result.ok:
        return f"{tool_name}: ok"
    reason = result.error_reason or "unknown error"
    return f"{tool_name}: failed ({reason})"


class AgentConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__("agent_node")
        self.declare_parameter("agent_streaming", True)
        self.declare_parameter("agent_verbose", True)
        configs = load_all_configs(node=self)
        self._prompts = configs.prompts
        self._trace_pub = create_trace_pub(self, configs.topics.trace.events)
        buffer_size = configs.thresholds.trace_buffer_size or 30
        self._trace_buffer = TraceBuffer(buffer_size)
        self._agent_factory = RosaAgentFactory(self)
        self._streaming_enabled = bool(self.get_parameter("agent_streaming").value)
        self._verbose_enabled = bool(self.get_parameter("agent_verbose").value)
        tool_callback = None if self._streaming_enabled else self._on_tool_result
        self._agent = self._agent_factory.create_agent(
            tool_callback=tool_callback,
            streaming=self._streaming_enabled,
            verbose=self._verbose_enabled,
        )
        self._tool_results: List[Tuple[str, ToolResult]] = []
        self._tool_results_lock = threading.Lock()
        self._memory = self._agent.memory
        self._stop_event = threading.Event()

    def run(self) -> None:
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        try:
            self._read_loop()
        finally:
            self._stop_event.set()
            self.destroy_node()

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = input("> ")
            except EOFError:
                break
            if not line.strip():
                continue
            command, args = parse_command(line)
            if command:
                if command == "help":
                    print(build_help_text(self._prompts.templates))
                    continue
                if command == "quit":
                    break
                if command == "status":
                    result = self._run_tool("get_status", tool_impl.get_status)
                    if result is not None:
                        self._maybe_explain_status(result, args)
                    continue
                if command == "cap":
                    self._run_tool("capture_and_score", tool_impl.capture_and_score)
                    continue
                if command == "nudge":
                    self._run_tool("move_nudge", tool_impl.move_nudge)
                    continue
                if command == "mast_open":
                    self._run_tool("mast_open", tool_impl.mast_open)
                    continue
                if command == "mast_close":
                    self._run_tool("mast_close", tool_impl.mast_close)
                    continue
                if command == "mast_rotate":
                    self._run_tool("mast_rotate", tool_impl.mast_rotate)
                    continue
                if command == "demo":
                    demo_text = resolve_demo_text(
                        self._prompts.templates, DEMO_TEMPLATE_KEY
                    )
                    if not demo_text:
                        print(f"No demo template '{DEMO_TEMPLATE_KEY}' configured.")
                        continue
                    self._run_prompt(demo_text)
                    continue
                print(f"Unknown command: {command}")
                continue
            self._run_prompt(args)

    def _run_prompt(self, text: str) -> None:
        with self._tool_results_lock:
            self._tool_results.clear()
        prompt = self._build_prompt(text)
        if self._streaming_enabled and hasattr(self._agent.agent, "astream"):
            try:
                asyncio.run(self._run_prompt_streaming(prompt))
            except Exception as exc:
                print(f"LLM error: {exc}")
            return
        try:
            response = self._agent.run_once(prompt)
        except Exception as exc:
            print(f"LLM error: {exc}")
            return
        llm_text = extract_llm_text(response)
        print(f"LLM: {llm_text}")
        summaries = self._consume_tool_summaries()
        if summaries:
            print("Tools:")
            for line in summaries:
                print(f"  {line}")

    async def _run_prompt_streaming(self, prompt: str) -> str:
        agent = self._agent.agent
        if not hasattr(agent, "astream"):
            raise RuntimeError("Agent does not support streaming")
        final_text = ""
        token_active = False
        token_seen = False
        def _print_stream_line(line: str) -> None:
            nonlocal token_active
            if token_active:
                print()
                token_active = False
            if self._verbose_enabled:
                print()
            print(line, flush=True)

        async for event in agent.astream(prompt):
            event_type = str(event.get("type", ""))
            if event_type == "token":
                content = str(event.get("content", ""))
                if content:
                    print(content, end="", flush=True)
                    token_active = True
                    token_seen = True
                continue
            if event_type == "tool_start":
                name = str(event.get("name", ""))
                _print_stream_line(f"Tool: {name} start")
                self._publish_stream_event(event)
                continue
            if event_type == "tool_end":
                name = str(event.get("name", ""))
                ok, error_reason = self._extract_stream_outcome(event.get("output"))
                if ok is True:
                    summary = "ok=true"
                elif ok is False:
                    detail = error_reason or "unknown error"
                    summary = f"ok=false reason={detail}"
                else:
                    summary = "ok=unknown"
                _print_stream_line(f"Tool: {name} end ({summary})")
                self._publish_stream_event(event)
                continue
            if event_type == "final":
                content = str(event.get("content", ""))
                final_text = content
                if token_seen or content.strip():
                    _print_stream_line(f"LLM final: {content}")
                self._publish_stream_event(event)
                continue
            if event_type == "error":
                content = str(event.get("content", ""))
                _print_stream_line(f"LLM error: {content}")
                self._publish_stream_event(event)
                continue
        if token_active:
            print()
        return final_text

    def _run_tool(self, tool_name: str, func: Any) -> Optional[ToolResult]:
        result = func(self)
        if not isinstance(result, ToolResult):
            print(f"{tool_name}: unexpected result type")
            return None
        event = build_tool_trace_event(tool_name, result)
        self._publish_event(event)
        summary = summarize_tool_result(tool_name, result)
        print(f"Tool: {summary}")
        return result

    def _maybe_explain_status(self, result: ToolResult, args: str) -> None:
        if not result.data:
            return
        status_text = json.dumps(result.data, ensure_ascii=False)
        print(f"Status: {status_text}")
        if args.lower() != "llm":
            return
        prompt = (
            "Explain the rover status in plain language. "
            "Do not call any tools. Use only this JSON:\n"
            f"{status_text}"
        )
        self._run_prompt(prompt)

    def _on_tool_result(self, tool_name: str, result: Any) -> None:
        if not isinstance(result, ToolResult):
            return
        event = build_tool_trace_event(tool_name, result)
        self._publish_event(event)
        with self._tool_results_lock:
            self._tool_results.append((tool_name, result))

    def _publish_event(self, event: TraceEvent) -> None:
        payload = encode_event(event)
        msg = String()
        msg.data = payload
        self._trace_pub.publish(msg)
        self._trace_buffer.append(event)
        if self._memory is not None:
            lines = self._trace_buffer.as_lines(1)
            if lines:
                self._memory.add(lines[0])

    def _build_prompt(self, text: str) -> str:
        if self._memory is None:
            return text
        recent = list(self._memory.recent())
        if not recent:
            return text
        joined = "\n".join(f"- {line}" for line in recent)
        return f"{text}\n\nRecent events:\n{joined}"

    def _consume_tool_summaries(self) -> List[str]:
        with self._tool_results_lock:
            results = list(self._tool_results)
            self._tool_results.clear()
        return [summarize_tool_result(name, result) for name, result in results]

    def _publish_stream_event(self, event: Dict[str, Any]) -> None:
        event_type = str(event.get("type", ""))
        kind = "OBSERVE"
        message = ""
        tool_name = None
        ok = None
        error_reason = None
        data = None

        if event_type == "tool_start":
            tool_name = str(event.get("name", ""))
            kind = "ACT"
            message = f"{tool_name} start"
            data = {"input": self._safe_stream_value(event.get("input"))}
        elif event_type == "tool_end":
            tool_name = str(event.get("name", ""))
            kind = "RESULT"
            message = f"{tool_name} end"
            ok, error_reason = self._extract_stream_outcome(event.get("output"))
            data = {"output": self._safe_stream_value(event.get("output"))}
        elif event_type == "final":
            kind = "RESULT"
            message = "llm final"
            data = {"content": self._safe_stream_value(event.get("content"))}
        elif event_type == "error":
            kind = "ERROR"
            message = "llm error"
            content = event.get("content")
            error_reason = str(content) if content is not None else "unknown error"
            data = {"content": self._safe_stream_value(content)}
        else:
            return

        trace_event = TraceEvent(
            event_id=str(uuid.uuid4()),
            ts=time.time(),
            kind=kind,
            message=message,
            tool_name=tool_name,
            ok=ok,
            error_reason=error_reason,
            data=data,
        )
        self._publish_event(trace_event)

    @staticmethod
    def _safe_stream_value(value: Any) -> str:
        if value is None:
            return ""
        if isinstance(value, (str, int, float, bool)):
            return str(value)
        try:
            return json.dumps(value, ensure_ascii=False)
        except TypeError:
            return repr(value)

    @staticmethod
    def _extract_stream_outcome(output: Any) -> Tuple[Optional[bool], Optional[str]]:
        if isinstance(output, ToolResult):
            return output.ok, output.error_reason
        if isinstance(output, dict):
            ok_value = output.get("ok")
            ok = ok_value if isinstance(ok_value, bool) else None
            error_reason = output.get("error_reason")
            if not isinstance(error_reason, str) or not error_reason.strip():
                error_reason = None
            return ok, error_reason
        return None, None


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AgentConsoleNode()
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
