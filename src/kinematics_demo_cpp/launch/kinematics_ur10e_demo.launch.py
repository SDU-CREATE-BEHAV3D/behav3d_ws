#!/usr/bin/env python3
          
# =============================================================================
#   ____  _____ _   _    ___     _______ ____  
#  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
#  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
#  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
#  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
#                                               
#                                               
# Author: Lucas José Helle <luh@iti.sdu.dk>
# Maintainers:
#   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
#   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
# Institute: University of Southern Denmark (Syddansk Universitet)
# Date: 2025-07
# =============================================================================

from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # -------------------------------------------------------------------------
    # 1) User‑overridable CLI arguments
    # -------------------------------------------------------------------------
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1",
        description="IP address of the UR controller (real robot).",
    )
    mock_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="true = simulation/mock, false = real hardware",
    )
    group_arg = DeclareLaunchArgument('group', default_value='ur_arm', description='MoveIt planning group')
    root_link_arg = DeclareLaunchArgument('root_link', default_value='world', description='Root/world link frame')
    eef_link_arg = DeclareLaunchArgument('eef_link', default_value='femto__depth_optical_frame', description='End-effector link frame')
    planning_pipeline_arg = DeclareLaunchArgument('planning_pipeline', default_value='pilz_industrial_motion_planner', description='Planning pipeline id')
    max_velocity_scale_arg = DeclareLaunchArgument('max_velocity_scale', default_value='0.5', description='Max velocity scale [0..1]')
    max_accel_scale_arg = DeclareLaunchArgument('max_accel_scale', default_value='0.5', description='Max acceleration scale [0..1]')
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Enable debug logging')

    # -------------------------------------------------------------------------
    # 2) Common paths
    # -------------------------------------------------------------------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell_moveit_config"), "launch"
    )

    # -------------------------------------------------------------------------
    # 3) UR driver (real robot or mock) Calling I40_workcell start_robot launch
    # -------------------------------------------------------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, "start_robot.launch.py")),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )
    # -------------------------------------------------------------------------
    # 4) MoveIt stack (Initialize I40_workspace_moveit_config movegroup)
    # -------------------------------------------------------------------------

    
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_launch_dir, "move_group.launch.py")),
    )
    # -------------------------------------------------------------------------
    # 5) Re‑build the *same* MoveIt config so we can share it with a helper node
    # -------------------------------------------------------------------------

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="i40_workcell_moveit_config")
        .robot_description_semantic(Path("config") / "ur.srdf")
        # .moveit_cpp(
        #     file_path=os.path.join(
        #         get_package_share_directory("pilz_demo"),
        #         "config/pilz_demo.yaml",
        #     )
        # )
        .to_moveit_configs()
    )
    # RViz
    rviz_config_file = (
        get_package_share_directory("i40_workcell_moveit_config") + "/config/move_group.rviz"
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

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="kinematics_demo_cpp",
        package="kinematics_demo_cpp",
        executable="demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                # These values populate NodeOptions-backed parameters used by PilzMotionController & MotionVisualizer
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

    return LaunchDescription(
        [
            robot_ip_arg,
            mock_arg,
            group_arg,
            root_link_arg,
            eef_link_arg,
            planning_pipeline_arg,
            max_velocity_scale_arg,
            max_accel_scale_arg,
            debug_arg,
            ur_driver,
            moveit_stack,
            rviz_node,
            move_group_demo,
        ]
    )