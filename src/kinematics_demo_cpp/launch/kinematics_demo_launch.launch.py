#!/usr/bin/env python3
#
# Launch UR‑driver, MoveIt, and the C++ demo node using PilzMotionController.
#
# Usage (simulation/mock by default):
#   ros2 launch kinematics_demo_cpp kinematics_demp_launc.launch.py
#
# Override to real robot:
#   ros2 launch kinematics_demo_cpp kinematics_demp_launc.launch.py robot_ip:=192.168.56.101 use_mock_hardware:=false
#
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():
    # --- CLI args ---------------------------------------------------------
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
    # --- Paths ------------------------------------------------------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell_moveit_config"), "launch"
    )
    # --- UR driver --------------------------------------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "start_robot.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )
    # --- MoveIt stack -----------------------------------------------------
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, "move_group.launch.py")
        ),
    )
    # --- MoveIt config (for demo node) -----------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="i40_workcell_moveit_config"
        )
        .robot_description_semantic(Path("config") / "ur.srdf")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("kinematics_demo_cpp"),
                "config/pilz_demo.yaml",
            )
        )
        .to_moveit_configs()
    )
    # --- C++ demo node ----------------------------------------------------
    demo_node = Node(
        package="kinematics_demo_cpp",
        executable="kinematics_demo_cpp_node",
        name="pilz_demo_cpp",
        output="both",
        parameters=[moveit_config.to_dict()],
    )
    # --- Assemble LD ------------------------------------------------------
    return LaunchDescription(
        [
            robot_ip_arg,
            mock_arg,
            ur_driver,
            moveit_stack,
            demo_node,
        ]
    )

