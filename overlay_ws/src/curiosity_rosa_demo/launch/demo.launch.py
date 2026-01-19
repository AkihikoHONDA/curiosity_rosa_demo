import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = LaunchConfiguration("config_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    debug = LaunchConfiguration("debug")

    default_config_dir = str(
        Path(get_package_share_directory("curiosity_rosa_demo")) / "config"
    )

    actions = [
        DeclareLaunchArgument(
            "config_dir",
            default_value=default_config_dir,
            description="Config directory containing YAML files.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2 if true.",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Enable continuous capture publish in simulator.",
        ),
    ]

    actions.append(
        Node(
            package="curiosity_rosa_demo",
            executable="simulator_node",
            parameters=[{"config_dir": config_dir, "debug": debug}],
        )
    )
    actions.append(
        Node(
            package="curiosity_rosa_demo",
            executable="adapter_node",
            parameters=[{"config_dir": config_dir}],
        )
    )
    actions.append(
        Node(
            package="curiosity_rosa_demo",
            executable="visualizer_node",
            parameters=[{"config_dir": config_dir}],
        )
    )

    actions.append(
        LogInfo(
            msg=(
                "agent_node is not launched by demo.launch.py. "
                "Start it in a separate terminal with the ROSA venv."
            )
        )
    )

    rviz_path = str(
        Path(get_package_share_directory("curiosity_rosa_demo"))
        / "config"
        / "rviz.rviz"
    )
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
            condition=IfCondition(use_rviz),
        )
    )

    return LaunchDescription(actions)
