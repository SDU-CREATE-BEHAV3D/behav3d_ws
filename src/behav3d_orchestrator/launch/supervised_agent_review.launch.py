"""Launch only review coordinator and evidence bridge; no workcell bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('behav3d_orchestrator')) /
                 'config' / 'supervised_agent_loop_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='behav3d_orchestrator', executable='observation_bridge',
             name='behav3d_observation_bridge', output='screen',
             parameters=[LaunchConfiguration('config')]),
        Node(package='behav3d_orchestrator', executable='supervised_agent_loop',
             name='behav3d_supervised_agent_loop', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
