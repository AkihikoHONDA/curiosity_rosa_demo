from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node

from curiosity_rosa_demo.agent.rosa_agent_factory import RosaAgentFactory, TOOL_NAMES


@pytest.fixture
def ros_node():
    rclpy.init()
    node = Node("test_rosa_factory")
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_factory_builds_stub_agent(ros_node: Node):
    config_dir = Path(__file__).resolve().parents[1] / "config"
    factory = RosaAgentFactory(ros_node, config_dir=config_dir)
    agent = factory.create_agent(use_stub=True)

    assert agent.tool_names == TOOL_NAMES
    result = agent.run_once("hello")
    assert result["prompt"] == "hello"
    assert result["tools"] == TOOL_NAMES
