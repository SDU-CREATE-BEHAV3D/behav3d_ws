"""Review-only startup nodes, separate from scanning/printing bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("behav3d_orchestrator"))
        / "config/initial_startup_config.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config),
            Node(
                package="behav3d_orchestrator",
                executable="initial_wall_backend",
                name="behav3d_initial_wall_backend",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
            Node(
                package="behav3d_orchestrator",
                executable="initial_startup",
                name="behav3d_initial_startup",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
