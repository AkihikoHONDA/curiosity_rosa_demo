from __future__ import annotations

import inspect
import os
from pathlib import Path
from typing import Any, Callable, List, Optional

from curiosity_rosa_demo.agent.memory import build_memory, ShortTermMemory
from curiosity_rosa_demo.infra.config_loader import PromptsConfig, load_all_configs
from curiosity_rosa_demo.tools import tool_impl

try:
    import rosa  # type: ignore
    from rosa import RobotSystemPrompts  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    rosa = None
    RobotSystemPrompts = None

try:
    from langchain.tools import tool as lc_tool  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    lc_tool = None


TOOL_NAMES = [
    "capture_and_score",
    "mast_open",
    "mast_close",
    "mast_rotate",
    "move_forward",
    "turn_left",
    "turn_right",
    "move_stop",
    "get_status",
]


class RosaAgentWrapper:
    def __init__(
        self,
        agent: Any,
        tool_names: List[str],
        memory: Optional[ShortTermMemory],
    ) -> None:
        self.agent = agent
        self.tool_names = tool_names
        self.memory = memory

    def run_once(self, prompt: str) -> Any:
        if hasattr(self.agent, "run"):
            return self.agent.run(prompt)
        if hasattr(self.agent, "invoke"):
            return self.agent.invoke(prompt)
        if callable(self.agent):
            return self.agent(prompt)
        raise RuntimeError("Agent does not support run/invoke/callable interface")


class StubAgent:
    def __init__(self, tool_names: List[str]) -> None:
        self.tool_names = tool_names

    def run(self, prompt: str) -> dict:
        return {"prompt": prompt, "tools": self.tool_names}


class RosaAgentFactory:
    def __init__(self, node: Any, *, config_dir: Optional[Path] = None) -> None:
        self.node = node
        configs = load_all_configs(config_dir=config_dir, node=node)
        self.prompts = configs.prompts
        self.tool_names = list(TOOL_NAMES)

    def create_agent(
        self,
        *,
        llm: Optional[Any] = None,
        use_stub: bool = False,
    ) -> RosaAgentWrapper:
        memory = build_memory(
            self.prompts.memory.enabled, self.prompts.memory.max_events
        )
        if use_stub:
            return RosaAgentWrapper(
                agent=StubAgent(self.tool_names),
                tool_names=self.tool_names,
                memory=memory,
            )

        llm = llm or self._build_llm()
        tools = self._build_tools()
        system_prompts = self._build_system_prompts(self.prompts)
        system_prompt_text = self._build_system_prompt_text(
            self.prompts, system_prompts
        )

        if rosa is None:
            raise RuntimeError(
                "ROSA package is not installed. Install rosa and try again."
            )

        agent = self._create_rosa_agent(
            llm=llm,
            tools=tools,
            system_prompts=system_prompts,
            system_prompt_text=system_prompt_text,
        )
        return RosaAgentWrapper(agent=agent, tool_names=self.tool_names, memory=memory)

    def _build_llm(self) -> Any:
        api_key = self._get_param_or_env("llm_api_key", "OPENAI_API_KEY")
        model = self._get_param_or_env("llm_model", "OPENAI_MODEL", required=False)
        if not api_key:
            raise RuntimeError(
                "LLM API key is missing. Set OPENAI_API_KEY or node param llm_api_key."
            )
        if rosa is None:
            raise RuntimeError(
                "ROSA package is not installed. Install rosa to use LLM."
            )
        if hasattr(rosa, "create_openai_llm"):
            return rosa.create_openai_llm(api_key=api_key, model=model)
        raise RuntimeError("ROSA LLM factory is unavailable in this environment.")

    def _get_param_or_env(
        self, param_name: str, env_name: str, *, required: bool = True
    ) -> Optional[str]:
        value = None
        try:
            param = self.node.get_parameter(param_name)
            if isinstance(param.value, str) and param.value:
                value = param.value
        except Exception:
            value = None
        if not value:
            value = os.getenv(env_name)
        if required and not value:
            return None
        return value

    def _build_tools(self) -> List[Any]:
        tool_funcs = {
            "capture_and_score": tool_impl.capture_and_score,
            "mast_open": tool_impl.mast_open,
            "mast_close": tool_impl.mast_close,
            "mast_rotate": tool_impl.mast_rotate,
            "move_forward": tool_impl.move_forward,
            "turn_left": tool_impl.turn_left,
            "turn_right": tool_impl.turn_right,
            "move_stop": tool_impl.move_stop,
            "get_status": tool_impl.get_status,
        }

        tools: List[Any] = []
        for name in self.tool_names:
            func = tool_funcs[name]
            tools.append(self._wrap_tool(name, func))
        return tools

    def _wrap_tool(self, name: str, func: Callable[..., Any]) -> Any:
        if lc_tool is None:
            return {"name": name, "func": func}
        return lc_tool(name)(lambda node=self.node, f=func: f(node))

    def _build_system_prompts(self, prompts: PromptsConfig):
        if RobotSystemPrompts is None:
            raise RuntimeError(
                "RobotSystemPrompts is unavailable. Install rosa to proceed."
            )
        return RobotSystemPrompts(
            embodiment_and_persona=prompts.robot_system_prompts.embodiment_and_persona,
            critical_instructions=prompts.robot_system_prompts.critical_instructions,
            relevant_context=prompts.robot_system_prompts.relevant_context,
            nuance_and_assumptions=prompts.robot_system_prompts.nuance_and_assumptions,
        )

    def _build_system_prompt_text(
        self, prompts: PromptsConfig, system_prompts: Any
    ) -> str:
        parts = [
            prompts.robot_system_prompts.embodiment_and_persona,
            prompts.robot_system_prompts.critical_instructions,
            prompts.robot_system_prompts.relevant_context,
            prompts.robot_system_prompts.nuance_and_assumptions,
        ]
        text = "\n\n".join(part.strip() for part in parts if part.strip())
        if prompts.bootstrap.enabled and prompts.bootstrap.text.strip():
            text = f"{text}\n\n{prompts.bootstrap.text.strip()}"
        return text

    def _create_rosa_agent(
        self,
        *,
        llm: Any,
        tools: List[Any],
        system_prompts: Any,
        system_prompt_text: str,
    ) -> Any:
        rosa_cls = getattr(rosa, "ROSA", None)
        if rosa_cls is None:
            raise RuntimeError("ROSA class is not available in rosa package.")
        sig = inspect.signature(rosa_cls)
        kwargs = {}
        for key, value in {
            "llm": llm,
            "tools": tools,
            "system_prompts": system_prompts,
            "system_prompt": system_prompt_text,
        }.items():
            if key in sig.parameters:
                kwargs[key] = value
        if not kwargs:
            raise RuntimeError("ROSA constructor does not accept known parameters.")
        return rosa_cls(**kwargs)
