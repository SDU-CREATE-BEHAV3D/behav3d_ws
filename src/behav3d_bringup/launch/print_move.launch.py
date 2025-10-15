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
    TimerAction,
)
from launch.conditions import IfCondition
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
    # MoveIt Params
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

    # Orbbec Params
    orbbec_enable_arg = DeclareLaunchArgument(
        "orbbec_enable",
        default_value="true",
        description="Start Orbbec camera (orbbec_camera/femto_bolt.launch.py)",
    )
    # MotionController Params
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
        default_value="femto_color_optical_calib",
        description="End-effector link"
    )

    planning_pipeline_arg = DeclareLaunchArgument(
        "planning_pipeline",
        default_value="pilz_industrial_motion_planner",
        description="Planning pipeline id"
    )

    max_velocity_scale_arg = DeclareLaunchArgument(
        "max_velocity_scale",
        default_value="0.2",
        description="Max velocity scale [0..1]"
    )

    max_accel_scale_arg = DeclareLaunchArgument(
        "max_accel_scale",
        default_value="0.2",
        description="Max acceleration scale [0..1]"
    )

    home_joints_deg_arg = DeclareLaunchArgument(
        "home_joints_deg",
        default_value="[-90.0, -120.0, 120.0, -90.0, 90.0, -150.0]",
        description="Home joint positions in degrees (list)"
    )
    # Misc
    
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable debug logging"
    )

    # -------------------------------------------------------------------------
    # 2) Common paths
    # -------------------------------------------------------------------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("ur20_workcell"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("ur20_workcell_moveit_config"), "launch"
    )
    orbbec_launch_dir = os.path.join(
        get_package_share_directory("orbbec_camera"), "launch"
    )

    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            orbbec_launch_dir, "femto_bolt.launch.py")),
        condition=IfCondition(LaunchConfiguration("orbbec_enable")),
        launch_arguments={
            # Color: 3840 x 2160 @ 30 fps
            "enable_color": "true",
            "color_width": "1920",
            "color_height": "1080",
            "color_fps": "30",
            "color_format": "MJPG",
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
    # 3) UR driver (real robot or mock) Calling ur20_workcell start_robot launch
    # -------------------------------------------------------------------------
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
    # -------------------------------------------------------------------------
    # 4) MoveIt stack (Initialize ur20_workspace_moveit_config movegroup)
    # -------------------------------------------------------------------------

    
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_launch_dir, "move_group.launch.py")),
    )
    # -------------------------------------------------------------------------
    # 5) Re‑build the *same* MoveIt config so we can share it with a helper node
    # -------------------------------------------------------------------------
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur20_workcell_moveit_config").to_moveit_configs()
        # Starts Pilz Industrial Motion Planner MoveGroupSequenceAction and MoveGroupSequenceService servers
    # RViz
    rviz_config_file = (
        get_package_share_directory("ur20_workcell_moveit_config") + "/config/move_group.rviz"
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
            moveit_config.joint_limits,
        ],
    )

    motion_bridge_node = Node(
        name="motion_bridge_node",
        package="behav3d_motion_bridge",
        executable="motion_bridge_node",
        output="screen",
        parameters=[
            # Pasa las descripciones de MoveIt, igual que haces con el demo
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    motion_bridge = TimerAction(period=2.0, actions=[motion_bridge_node])

    print_node= Node(
        name="behav3d_print",
        package="behav3d_print",
        executable="print_node",
        output="screen",
    )
    node_demo = Node(
        name="behav3d_demo",
        package="behav3d_demo",
        executable="mancap",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                # These values populate the NodeOptions-backed parameters used by PilzMotionController & MotionVisualizer
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
            orbbec_enable_arg,
            
            ur_driver,
            moveit_stack,
            orbbec_camera,

            rviz_node,
            motion_bridge,
            print_node,
       #    node_demo
        ]
    )