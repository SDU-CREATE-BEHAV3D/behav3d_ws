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
from launch_ros.parameter_descriptions import ParameterValue


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

    group_arg = DeclareLaunchArgument(
        "group",
        default_value="ur_arm",
        description="MoveIt planning group"
    )
    
    root_link_arg = DeclareLaunchArgument(
        "root_link",
        default_value="world",
        description="Root/world link frame"
    )
    
    eef_link_arg = DeclareLaunchArgument(
        "eef_link",
        default_value="femto__depth_optical_frame",
        description="End-effector link"
    )
    
    planning_pipeline_arg = DeclareLaunchArgument(
        "planning_pipeline",
        default_value="pilz_industrial_motion_planner",
        description="Planning pipeline id"
    )
    
    max_velocity_scale_arg = DeclareLaunchArgument(
        "max_velocity_scale",
        default_value="0.5",
        description="Max velocity scale [0..1]"
    )
    
    max_accel_scale_arg = DeclareLaunchArgument(
        "max_accel_scale",
        default_value="0.5",
        description="Max acceleration scale [0..1]"
    )
    
    robot_prefix_arg = DeclareLaunchArgument(
        "robot_prefix",
        default_value="ur10e",
        description="Robot prefix for link names (e.g. ur10e -> ur10e_tool0)"
    )
    
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="~/behav3d_ws/captures",
        description="Root output directory for sessions"
    )
    
    capture_delay_sec_arg = DeclareLaunchArgument(
        "capture_delay_sec",
        default_value="0.5",
        description="Wait time before capture [s]"
    )
    
    calib_timeout_sec_arg = DeclareLaunchArgument(
        "calib_timeout_sec",
        default_value="2.0",
        description="Calibration timeout [s]"
    )
    
    home_joints_deg_arg = DeclareLaunchArgument(
        "home_joints_deg",
        default_value="[45.0, -120.0, 120.0, -90.0, 90.0, -180.0]",
        description="Home joint positions in degrees (list)",
    )
    
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Enable debug logging"
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
            # Color: 3840 x 2160 @ 30 fps
            # "enable_color": "true",
            # "color_width": "3840",
            # "color_height": "2160",
            # "color_fps": "30",
            # "color_format": "MJPG",
            # Depth (NFOV, unbinned-equivalent): 640 x 576 @ 30 fps
            "enable_depth": "true",
            "depth_width": "640",
            "depth_height": "576",
            "depth_fps": "30",
            "depth_format": "Y16",
            # IR: 640 x 576 @ 30 fps
            "enable_ir": "true",
            "ir_width": "640",
            "ir_height": "576",
            "ir_fps": "30",
            "ir_format": "Y16"
            # PointCloud
            # "enable_point_cloud" : "false"
            # TODO: 'enable_ldp' throws compilation error!
            # Laser Dot Projector (true for scan / false for calibration)
            # "enable_ldp": "false"
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
    
    # ---------------------------------------------------------------------
    # How to override NodeOptions-backed parameters
    # From launch:
    #   ros2 launch behav3d_demo behav3d_demo_launch.launch.py \
    #     group:=ur_arm root_link:=world eef_link:=femto__depth_optical_frame \
    #     planning_pipeline:=pilz_industrial_motion_planner \
    #     max_velocity_scale:=0.35 max_accel_scale:=0.25 debug:=true \
    #     robot_prefix:=ur10e output_dir:=~/behav3d_ws/captures \
    #     capture_delay_sec:=0.6 calib_timeout_sec:=2.0
    # Directly (no launch):
    #   ros2 run behav3d_demo demo --ros-args \
    #     -p group:=ur_arm -p root_link:=world -p eef_link:=femto__depth_optical_frame \
    #     -p planning_pipeline:=pilz_industrial_motion_planner \
    #     -p max_velocity_scale:=0.35 -p max_accel_scale:=0.25 -p debug:=true \
    #     -p robot_prefix:=ur10e -p output_dir:=~/behav3d_ws/captures \
    #     -p capture_delay_sec:=0.6 -p calib_timeout_sec:=2.0

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
            {
                # These values populate the NodeOptions-backed parameters used by PilzMotionController and other nodes
                'group': LaunchConfiguration('group'),
                'root_link': LaunchConfiguration('root_link'),
                'eef_link': LaunchConfiguration('eef_link'),
                'planning_pipeline': LaunchConfiguration('planning_pipeline'),
                'max_velocity_scale': ParameterValue(LaunchConfiguration('max_velocity_scale'), value_type=float),
                'max_accel_scale': ParameterValue(LaunchConfiguration('max_accel_scale'), value_type=float),
                'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
                # SessionManager & Demo
                'robot_prefix': LaunchConfiguration('robot_prefix'),
                'output_dir': LaunchConfiguration('output_dir'),
                'capture_delay_sec': ParameterValue(LaunchConfiguration('capture_delay_sec'), value_type=float),
                'calib_timeout_sec': ParameterValue(LaunchConfiguration('calib_timeout_sec'), value_type=float),
                'home_joints_deg': LaunchConfiguration('home_joints_deg'),
            }
        ],
    )

    return LaunchDescription(
        [
            # Declare all CLI arguments first (safer resolution for LaunchConfiguration substitutions)
            robot_ip_arg,
            mock_arg,
            orbbec_enable_arg,
            group_arg,
            root_link_arg,
            eef_link_arg,
            planning_pipeline_arg,
            max_velocity_scale_arg,
            max_accel_scale_arg,
            robot_prefix_arg,
            output_dir_arg,
            capture_delay_sec_arg,
            calib_timeout_sec_arg,
            home_joints_deg_arg,
            debug_arg,
            # Then include/launch nodes
            ur_driver,
            moveit_stack,
            orbbec_camera,
            rviz_node,
            move_group_demo
        ]
    )