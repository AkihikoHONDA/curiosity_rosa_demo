from __future__ import annotations

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
        configs = load_all_configs(node=self)
        self._prompts = configs.prompts
        self._trace_pub = create_trace_pub(self, configs.topics.trace.events)
        buffer_size = configs.thresholds.trace_buffer_size or 30
        self._trace_buffer = TraceBuffer(buffer_size)
        self._agent_factory = RosaAgentFactory(self)
        self._agent = self._agent_factory.create_agent(
            tool_callback=self._on_tool_result
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
                    self._run_tool("get_status", tool_impl.get_status)
                    continue
                if command == "cap":
                    self._run_tool("capture_and_score", tool_impl.capture_and_score)
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

    def _run_tool(self, tool_name: str, func: Any) -> None:
        result = func(self)
        if not isinstance(result, ToolResult):
            print(f"{tool_name}: unexpected result type")
            return
        event = build_tool_trace_event(tool_name, result)
        self._publish_event(event)
        summary = summarize_tool_result(tool_name, result)
        print(f"Tool: {summary}")

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


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AgentConsoleNode()
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
