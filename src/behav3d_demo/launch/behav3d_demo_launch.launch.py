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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    orbbec_enable_arg = DeclareLaunchArgument(
        "orbbec_enable",
        default_value="true",
        description="Start Orbbec camera (orbbec_camera/femto_bolt.launch.py)",
    )

    # -------------------------------------------------------------------------
    # 2) Common paths
    # -------------------------------------------------------------------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell_moveit_config"), "launch"
    )

    orbbec_launch_dir = os.path.join(
        get_package_share_directory("orbbec_camera"), "launch"
    )

    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(orbbec_launch_dir, "femto_bolt.launch.py")),
        condition=IfCondition(LaunchConfiguration("orbbec_enable")),
        launch_arguments={
            # Add camera-specific arguments here if you want to override defaults.
            # Example (only if supported by the Orbbec launch):
            # "device_index": "0",
            # "enable_color": "true",
            # "enable_depth": "true",
            # "enable_ir": "true",
        }.items(),
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
        get_package_share_directory("viz_demo") + "/launch/move_group.rviz"
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
        name="behav3d_demo",
        package="behav3d_demo",
        executable="demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            mock_arg,
            orbbec_enable_arg,
            ur_driver,
            moveit_stack,
            orbbec_camera,
            rviz_node,
            move_group_demo
        ]
    )