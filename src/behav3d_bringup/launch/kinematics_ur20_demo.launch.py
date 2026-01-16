#!/usr/bin/env python3
from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # CLI args
    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="127.0.0.1")
    mock_arg = DeclareLaunchArgument("use_mock_hardware", default_value="true")
    group_arg = DeclareLaunchArgument("group", default_value="ur_arm")
    root_link_arg = DeclareLaunchArgument("root_link", default_value="world")
    eef_link_arg = DeclareLaunchArgument("eef_link", default_value="femto__depth_optical_frame")
    planning_pipeline_arg = DeclareLaunchArgument("planning_pipeline", default_value="ompl")
    max_velocity_scale_arg = DeclareLaunchArgument("max_velocity_scale", default_value="0.5")
    max_accel_scale_arg = DeclareLaunchArgument("max_accel_scale", default_value="0.5")
    debug_arg = DeclareLaunchArgument("debug", default_value="false")
    run_demo_arg = DeclareLaunchArgument("run_demo", default_value="false",
                                         description="Start kinematics_demo_cpp if available")

    # Paths
    ur_launch_dir = os.path.join(get_package_share_directory("ur20_workcell"), "launch")
    moveit_launch_dir = os.path.join(get_package_share_directory("ur20_workcell_moveit_config"), "launch")

    # UR driver
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, "start_robot.launch.py")),
        launch_arguments={
            "ur_type": "ur20",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # MoveIt (publish robot_description & SRDF so RViz and others get them)
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_launch_dir, "move_group.launch.py")),
        launch_arguments={
            "publish_robot_description": "true",
            "publish_robot_description_semantic": "true",
            "allow_trajectory_execution": "true",
            "capabilities": "",
            "disable_capabilities": "",
            "planning_pipeline": LaunchConfiguration("planning_pipeline"),
        }.items(),
    )

    # Build the same MoveIt config for RViz params
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur20_workcell_moveit_config")
        .robot_description_semantic(Path("config") / "ur.srdf")
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("ur20_workcell_moveit_config"), "config", "move_group.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Optionally add the demo node if the package exists AND run_demo:=true
    extra_nodes = []
    want_demo = LaunchConfiguration("run_demo")
    try:
        # check presence; will raise if missing
        get_package_share_directory("kinematics_demo_cpp")
        have_demo = True
    except PackageNotFoundError:
        have_demo = False

    if have_demo:
        # Note: LaunchCondition needs launch.conditions.IfCondition
        from launch.conditions import IfCondition
        from launch_ros.parameter_descriptions import ParameterValue
        extra_nodes.append(
            Node(
                name="kinematics_demo_cpp",
                package="kinematics_demo_cpp",
                executable="demo",
                output="screen",
                condition=IfCondition(want_demo),
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {
                        'group': LaunchConfiguration('group'),
                        'root_link': LaunchConfiguration('root_link'),
                        'eef_link': LaunchConfiguration('eef_link'),
                        'planning_pipeline': LaunchConfiguration('planning_pipeline'),
                        'max_velocity_scale': ParameterValue(LaunchConfiguration('max_velocity_scale'), value_type=float),
                        'max_accel_scale': ParameterValue(LaunchConfiguration('max_accel_scale'), value_type=float),
                        'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
                    }
                ],
            )
        )

    return LaunchDescription([
        robot_ip_arg, mock_arg, group_arg, root_link_arg, eef_link_arg,
        planning_pipeline_arg, max_velocity_scale_arg, max_accel_scale_arg,
        debug_arg, run_demo_arg,
        ur_driver,
        moveit_stack,
        rviz_node,
        *extra_nodes,
    ])
