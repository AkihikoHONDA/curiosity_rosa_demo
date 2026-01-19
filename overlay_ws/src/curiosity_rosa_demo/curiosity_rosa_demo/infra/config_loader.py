from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


@dataclass(frozen=True)
class ServiceConfig:
    kind: str
    name: str
    type: Optional[str]


@dataclass(frozen=True)
class TopicsImagesConfig:
    input_compressed: str
    output_capture_raw: str


@dataclass(frozen=True)
class TopicsTraceConfig:
    events: str


@dataclass(frozen=True)
class TopicsTfConfig:
    world_frame: str
    base_frame: str


@dataclass(frozen=True)
class TopicsServicesConfig:
    capture_and_score: ServiceConfig


@dataclass(frozen=True)
class TopicsAdapterConfig:
    move_forward: ServiceConfig
    turn_left: ServiceConfig
    turn_right: ServiceConfig
    move_stop: ServiceConfig
    mast_open: ServiceConfig
    mast_close: ServiceConfig
    mast_rotate: ServiceConfig
    get_status: ServiceConfig


@dataclass(frozen=True)
class TopicsCuriosityConfig:
    move_forward: ServiceConfig
    turn_left: ServiceConfig
    turn_right: ServiceConfig
    move_stop: ServiceConfig
    mast_open: ServiceConfig
    mast_close: ServiceConfig
    mast_rotate: ServiceConfig


@dataclass(frozen=True)
class TopicsVizConfig:
    marker_array: str


@dataclass(frozen=True)
class TopicsConfig:
    images: TopicsImagesConfig
    trace: TopicsTraceConfig
    tf: TopicsTfConfig
    services: TopicsServicesConfig
    adapter: TopicsAdapterConfig
    curiosity: TopicsCuriosityConfig
    viz: TopicsVizConfig


@dataclass(frozen=True)
class ThresholdsConfig:
    light_model_x_min: float
    light_model_x_good: float
    quality_score_threshold: float
    trace_buffer_size: Optional[int]
    viz_bright_zone_x_min: Optional[float]
    viz_bright_zone_x_max: Optional[float]


@dataclass(frozen=True)
class ToolCostsConfig:
    tools: Dict[str, int]


@dataclass(frozen=True)
class RobotSystemPromptsConfig:
    embodiment_and_persona: str
    critical_instructions: str
    relevant_context: str
    nuance_and_assumptions: str


@dataclass(frozen=True)
class BootstrapConfig:
    enabled: bool
    text: str


@dataclass(frozen=True)
class MemoryConfig:
    enabled: bool
    max_events: int


@dataclass(frozen=True)
class PromptsConfig:
    robot_system_prompts: RobotSystemPromptsConfig
    bootstrap: BootstrapConfig
    memory: MemoryConfig
    templates: Dict[str, Any]


@dataclass(frozen=True)
class RvizConfig:
    raw: Dict[str, Any]


@dataclass(frozen=True)
class ConfigBundle:
    topics: TopicsConfig
    thresholds: ThresholdsConfig
    tool_costs: ToolCostsConfig
    prompts: PromptsConfig
    rviz: RvizConfig


def resolve_config_dir(node: Any | None, override: str | None = None) -> Path:
    """Resolve config directory path.

    Priority:
    1) explicit override argument
    2) ROS2 parameter "config_dir" on the provided node (string)
    3) ament share directory: <share>/config
    """
    if override:
        return Path(override)

    if node is not None:
        try:
            parameter = node.get_parameter("config_dir")
            value = parameter.value
            if isinstance(value, str) and value:
                return Path(value)
        except Exception:
            pass

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise RuntimeError(
            "ament_index_python is required to resolve the default config directory. "
            "Provide config_dir explicitly or install the dependency."
        ) from exc

    share_dir = Path(get_package_share_directory("curiosity_rosa_demo"))
    return share_dir / "config"


def load_all_configs(config_dir: Path | None = None, node: Any | None = None) -> ConfigBundle:
    cfg_dir = config_dir or resolve_config_dir(node)
    topics = load_topics_config(cfg_dir / "topics.yaml")
    thresholds = load_thresholds_config(cfg_dir / "thresholds.yaml")
    tool_costs = load_tool_costs_config(cfg_dir / "tool_costs.yaml")
    prompts = load_prompts_config(cfg_dir / "prompts.yaml")
    rviz = load_rviz_config(cfg_dir / "rviz.yaml")
    return ConfigBundle(
        topics=topics,
        thresholds=thresholds,
        tool_costs=tool_costs,
        prompts=prompts,
        rviz=rviz,
    )


def load_topics_config(path: Path) -> TopicsConfig:
    data = _load_yaml(path)
    images = _require_dict(data, "images", path)
    trace = _require_dict(data, "trace", path)
    tf = _require_dict(data, "tf", path)
    services = _require_dict(data, "services", path)
    adapter = _require_dict(data, "adapter", path)
    curiosity = _require_dict(data, "curiosity", path)
    viz = _require_dict(data, "viz", path)

    images_cfg = TopicsImagesConfig(
        input_compressed=_require_str(images, "input_compressed", path, "images"),
        output_capture_raw=_require_str(
            images, "output_capture_raw", path, "images"
        ),
    )
    trace_cfg = TopicsTraceConfig(events=_require_str(trace, "events", path, "trace"))
    tf_cfg = TopicsTfConfig(
        world_frame=_require_str(tf, "world_frame", path, "tf"),
        base_frame=_require_str(tf, "base_frame", path, "tf"),
    )
    services_cfg = TopicsServicesConfig(
        capture_and_score=_require_service_config(
            services, "capture_and_score", path, allow_type_missing=True
        )
    )
    adapter_cfg = TopicsAdapterConfig(
        move_forward=_require_service_config(adapter, "move_forward", path),
        turn_left=_require_service_config(adapter, "turn_left", path),
        turn_right=_require_service_config(adapter, "turn_right", path),
        move_stop=_require_service_config(adapter, "move_stop", path),
        mast_open=_require_service_config(adapter, "mast_open", path),
        mast_close=_require_service_config(adapter, "mast_close", path),
        mast_rotate=_require_service_config(adapter, "mast_rotate", path),
        get_status=_require_service_config(adapter, "get_status", path),
    )
    curiosity_cfg = TopicsCuriosityConfig(
        move_forward=_require_service_config(curiosity, "move_forward", path),
        turn_left=_require_service_config(curiosity, "turn_left", path),
        turn_right=_require_service_config(curiosity, "turn_right", path),
        move_stop=_require_service_config(curiosity, "move_stop", path),
        mast_open=_require_service_config(curiosity, "mast_open", path),
        mast_close=_require_service_config(curiosity, "mast_close", path),
        mast_rotate=_require_service_config(curiosity, "mast_rotate", path),
    )
    viz_cfg = TopicsVizConfig(
        marker_array=_require_str(viz, "marker_array", path, "viz")
    )

    _validate_service_types(adapter_cfg, "std_srvs/srv/Trigger", path, "adapter")
    _validate_service_types(curiosity_cfg, "std_srvs/srv/Empty", path, "curiosity")

    return TopicsConfig(
        images=images_cfg,
        trace=trace_cfg,
        tf=tf_cfg,
        services=services_cfg,
        adapter=adapter_cfg,
        curiosity=curiosity_cfg,
        viz=viz_cfg,
    )


def load_thresholds_config(path: Path) -> ThresholdsConfig:
    data = _load_yaml(path)
    light_model = _require_dict(data, "light_model", path)
    quality = _require_dict(data, "quality", path)

    x_min = _require_number(light_model, "x_min", path, "light_model")
    x_good = _require_number(light_model, "x_good", path, "light_model")
    score_threshold = _require_number(quality, "score_threshold", path, "quality")
    if not 0.0 <= score_threshold <= 1.0:
        raise ValueError(
            f"{path}: quality.score_threshold must be between 0.0 and 1.0"
        )

    trace_buffer_size = _optional_int(data, "trace", "buffer_size", path)
    viz_min = _optional_number(data, "viz", "bright_zone_x_min", path)
    viz_max = _optional_number(data, "viz", "bright_zone_x_max", path)

    return ThresholdsConfig(
        light_model_x_min=x_min,
        light_model_x_good=x_good,
        quality_score_threshold=score_threshold,
        trace_buffer_size=trace_buffer_size,
        viz_bright_zone_x_min=viz_min,
        viz_bright_zone_x_max=viz_max,
    )


def load_tool_costs_config(path: Path) -> ToolCostsConfig:
    data = _load_yaml(path)
    tools = _require_dict(data, "tools", path)
    parsed: Dict[str, int] = {}
    for name, value in tools.items():
        if not isinstance(name, str):
            raise ValueError(f"{path}: tools key must be string")
        if not isinstance(value, int) or value < 0:
            raise ValueError(
                f"{path}: tools.{name} must be a non-negative integer"
            )
        parsed[name] = value
    return ToolCostsConfig(tools=parsed)


def load_prompts_config(path: Path) -> PromptsConfig:
    data = _load_yaml(path)
    prompts = _require_dict(data, "robot_system_prompts", path)
    bootstrap = _require_dict(data, "bootstrap", path)
    memory = _require_dict(data, "memory", path)
    templates = _require_dict(data, "templates", path)

    robot_prompts = RobotSystemPromptsConfig(
        embodiment_and_persona=_require_str(
            prompts, "embodiment_and_persona", path, "robot_system_prompts"
        ),
        critical_instructions=_require_str(
            prompts, "critical_instructions", path, "robot_system_prompts"
        ),
        relevant_context=_require_str(
            prompts, "relevant_context", path, "robot_system_prompts"
        ),
        nuance_and_assumptions=_require_str(
            prompts, "nuance_and_assumptions", path, "robot_system_prompts"
        ),
    )
    bootstrap_cfg = BootstrapConfig(
        enabled=_require_bool(bootstrap, "enabled", path, "bootstrap"),
        text=_require_str(bootstrap, "text", path, "bootstrap"),
    )
    memory_cfg = MemoryConfig(
        enabled=_require_bool(memory, "enabled", path, "memory"),
        max_events=_require_int(memory, "max_events", path, "memory"),
    )

    return PromptsConfig(
        robot_system_prompts=robot_prompts,
        bootstrap=bootstrap_cfg,
        memory=memory_cfg,
        templates=templates,
    )


def load_rviz_config(path: Path) -> RvizConfig:
    data = _load_yaml(path)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: rviz config must be a mapping")
    return RvizConfig(raw=data)


def _load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"{path} not found")
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: invalid YAML") from exc
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: root must be a mapping")
    return data


def _require_dict(data: Dict[str, Any], key: str, path: Path) -> Dict[str, Any]:
    if key not in data:
        raise ValueError(f"{path}: missing key '{key}'")
    value = data[key]
    if not isinstance(value, dict):
        raise ValueError(f"{path}: '{key}' must be a mapping")
    return value


def _require_str(data: Dict[str, Any], key: str, path: Path, parent: str) -> str:
    if key not in data:
        raise ValueError(f"{path}: missing key '{parent}.{key}'")
    value = data[key]
    if not isinstance(value, str):
        raise ValueError(f"{path}: '{parent}.{key}' must be a string")
    return value


def _require_int(data: Dict[str, Any], key: str, path: Path, parent: str) -> int:
    if key not in data:
        raise ValueError(f"{path}: missing key '{parent}.{key}'")
    value = data[key]
    if not isinstance(value, int):
        raise ValueError(f"{path}: '{parent}.{key}' must be an integer")
    return value


def _require_bool(data: Dict[str, Any], key: str, path: Path, parent: str) -> bool:
    if key not in data:
        raise ValueError(f"{path}: missing key '{parent}.{key}'")
    value = data[key]
    if not isinstance(value, bool):
        raise ValueError(f"{path}: '{parent}.{key}' must be a boolean")
    return value


def _require_number(data: Dict[str, Any], key: str, path: Path, parent: str) -> float:
    if key not in data:
        raise ValueError(f"{path}: missing key '{parent}.{key}'")
    value = data[key]
    if not isinstance(value, (int, float)):
        raise ValueError(f"{path}: '{parent}.{key}' must be a number")
    return float(value)


def _optional_int(
    data: Dict[str, Any], key: str, subkey: str, path: Path
) -> Optional[int]:
    if key not in data:
        return None
    section = data[key]
    if not isinstance(section, dict):
        raise ValueError(f"{path}: '{key}' must be a mapping")
    if subkey not in section:
        return None
    value = section[subkey]
    if not isinstance(value, int):
        raise ValueError(f"{path}: '{key}.{subkey}' must be an integer")
    return value


def _optional_number(
    data: Dict[str, Any], key: str, subkey: str, path: Path
) -> Optional[float]:
    if key not in data:
        return None
    section = data[key]
    if not isinstance(section, dict):
        raise ValueError(f"{path}: '{key}' must be a mapping")
    if subkey not in section:
        return None
    value = section[subkey]
    if not isinstance(value, (int, float)):
        raise ValueError(f"{path}: '{key}.{subkey}' must be a number")
    return float(value)


def _require_service_config(
    data: Dict[str, Any],
    key: str,
    path: Path,
    allow_type_missing: bool = False,
) -> ServiceConfig:
    entry = _require_dict(data, key, path)
    kind = _require_str(entry, "kind", path, key)
    name = _require_str(entry, "name", path, key)
    type_value = entry.get("type")
    if type_value is None and allow_type_missing:
        return ServiceConfig(kind=kind, name=name, type=None)
    if type_value is None:
        raise ValueError(f"{path}: missing key '{key}.type'")
    if not isinstance(type_value, str):
        raise ValueError(f"{path}: '{key}.type' must be a string")
    return ServiceConfig(kind=kind, name=name, type=type_value)


def _validate_service_types(
    config: Any, expected_type: str, path: Path, section: str
) -> None:
    for field_name in config.__dataclass_fields__:
        service = getattr(config, field_name)
        if service.type is None:
            continue
        if service.type != expected_type:
            raise ValueError(
                f"{path}: {section}.{field_name}.type must be '{expected_type}'"
            )
