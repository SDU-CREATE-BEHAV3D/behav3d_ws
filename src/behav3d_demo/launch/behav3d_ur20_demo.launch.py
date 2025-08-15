#!/usr/bin/env python3

# =============================================================================
#   ____  _____ _   _    ___     _______ ____
#  | __ )| ____| | | |  / \ \   / /___ /|  _ \
#  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
#  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
#  |____/|_____|_| |_/_/   \_\_/  |____/|____/
#
#
# Author: Lucas Helle Pessot <luh@iti.sdu.dk>
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

    # SessionManager Params
    robot_prefix_arg = DeclareLaunchArgument(
        "robot_prefix",
        default_value="ur20",
        description="Robot prefix for link names (e.g. ur20 -> ur20_tool0)"
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="~/behav3d_ws/captures",
        description="Root output directory for sessions"
    )

    capture_delay_sec_arg = DeclareLaunchArgument(
        "capture_delay_sec",
        default_value="2.0",
        description="Wait time before capture [s]"
    )

    calib_timeout_sec_arg = DeclareLaunchArgument(
        "calib_timeout_sec",
        default_value="2.0",
        description="Calibration timeout [s]"
    )

    # Hand-eye Params

    handeye_session_dir_arg = DeclareLaunchArgument(
        'handeye_session_dir',
        default_value='',
        description='Absolute path to a specific hand-eye session directory.'
    )

    handeye_output_dir_arg = DeclareLaunchArgument(
        'handeye_output_dir',
        default_value='~/behav3d_ws/captures',
        description='Root containing session-* folders for hand-eye.'
    )

    handeye_board_squares_x_arg = DeclareLaunchArgument(
        'handeye_board_squares_x',
        default_value='9',
        description='Charuco squares X for hand-eye'
    )

    handeye_board_squares_y_arg = DeclareLaunchArgument(
        'handeye_board_squares_y',
        default_value='12',
        description='Charuco squares Y for hand-eye'
    )

    handeye_square_length_m_arg = DeclareLaunchArgument(
        'handeye_square_length_m',
        default_value='0.03',
        description='Square length (m) for hand-eye'
    )

    handeye_marker_length_m_arg = DeclareLaunchArgument(
        'handeye_marker_length_m',
        default_value='0.022',
        description='Marker length (m) for hand-eye'
    )

    handeye_aruco_dict_arg = DeclareLaunchArgument(
        'handeye_aruco_dict',
        default_value='DICT_5X5_100',
        description='ArUco dictionary name for hand-eye (e.g., "DICT_5X5_1000").'
    )

    handeye_calibration_method_arg = DeclareLaunchArgument(
        'handeye_calibration_method',
        default_value='tsai',
        description='Hand-eye method'
    )

    handeye_visualize_pause_ms_arg = DeclareLaunchArgument(
        'handeye_visualize_pause_ms',
        default_value='1000',
        description='Wait time for visualization window in milliseconds.'
    )

    handeye_visualize_arg = DeclareLaunchArgument(
        'handeye_visualize',
        default_value='true',
        description='Enable visualization for hand-eye (Charuco overlays, axes)'
    )
    handeye_visualize_display_scale_arg = DeclareLaunchArgument(
        'handeye_visualize_display_scale',
        default_value='0.5',
        description='Uniform scale factor for on-screen visualization (0.1–4.0)'
    )
    handeye_ir_clip_max_arg = DeclareLaunchArgument(
        'handeye_ir_clip_max',
        default_value='-1',
        description='If >0, clamp 16-bit IR to this value before normalization.'
    )
    handeye_ir_invert_arg = DeclareLaunchArgument(
        'handeye_ir_invert',
        default_value='false',
        description='Invert IR image after normalization.'
    )
    handeye_ir_use_clahe_arg = DeclareLaunchArgument(
        'handeye_ir_use_clahe',
        default_value='true',
        description='Apply CLAHE to IR image.'
    )
    handeye_ir_clahe_clip_arg = DeclareLaunchArgument(
        'handeye_ir_clahe_clip',
        default_value='3.0',
        description='CLAHE clip limit for IR.'
    )
    handeye_ir_clahe_tiles_arg = DeclareLaunchArgument(
        'handeye_ir_clahe_tiles',
        default_value='8',
        description='CLAHE tile grid size (NxN) for IR.'
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
            "color_width": "3840",
            "color_height": "2160",
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
        PythonLaunchDescriptionSource(os.path.join(
            ur_launch_dir, "start_robot.launch.py")),
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
        PythonLaunchDescriptionSource(os.path.join(
            moveit_launch_dir, "move_group.launch.py")),
    )
    # -------------------------------------------------------------------------
    # 5) Re‑build the *same* MoveIt config so we can share it with a helper node
    # -------------------------------------------------------------------------

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="ur20_workcell_moveit_config")
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
        get_package_share_directory(
            "ur20_workcell_moveit_config") + "/config/move_group.rviz"
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
    #     robot_prefix:=ur20 output_dir:=~/behav3d_ws/captures \
    #     capture_delay_sec:=0.6 calib_timeout_sec:=2.0
    # Directly (no launch):
    #   ros2 run behav3d_demo demo --ros-args \
    #     -p group:=ur_arm -p root_link:=world -p eef_link:=femto__depth_optical_frame \
    #     -p planning_pipeline:=pilz_industrial_motion_planner \
    #     -p max_velocity_scale:=0.35 -p max_accel_scale:=0.25 -p debug:=true \
    #     -p robot_prefix:=ur20 -p output_dir:=~/behav3d_ws/captures \
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
                # Handeye parameters
                'handeye_session_dir': LaunchConfiguration('handeye_session_dir'),
                'handeye_output_dir': LaunchConfiguration('handeye_output_dir'),
                'handeye_board_squares_x': ParameterValue(LaunchConfiguration('handeye_board_squares_x'), value_type=int),
                'handeye_board_squares_y': ParameterValue(LaunchConfiguration('handeye_board_squares_y'), value_type=int),
                'handeye_square_length_m': ParameterValue(LaunchConfiguration('handeye_square_length_m'), value_type=float),
                'handeye_marker_length_m': ParameterValue(LaunchConfiguration('handeye_marker_length_m'), value_type=float),
                'handeye_aruco_dict': LaunchConfiguration('handeye_aruco_dict'),
                'handeye_calibration_method': LaunchConfiguration('handeye_calibration_method'),
                'handeye_visualize_pause_ms': ParameterValue(LaunchConfiguration('handeye_visualize_pause_ms'), value_type=int),
                'handeye_visualize': ParameterValue(LaunchConfiguration('handeye_visualize'), value_type=bool),
                'handeye_visualize_display_scale': ParameterValue(LaunchConfiguration('handeye_visualize_display_scale'), value_type=float),
                'handeye_ir_clip_max': ParameterValue(LaunchConfiguration('handeye_ir_clip_max'), value_type=int),
                'handeye_ir_invert': ParameterValue(LaunchConfiguration('handeye_ir_invert'), value_type=bool),
                'handeye_ir_use_clahe': ParameterValue(LaunchConfiguration('handeye_ir_use_clahe'), value_type=bool),
                'handeye_ir_clahe_clip': ParameterValue(LaunchConfiguration('handeye_ir_clahe_clip'), value_type=float),
                'handeye_ir_clahe_tiles': ParameterValue(LaunchConfiguration('handeye_ir_clahe_tiles'), value_type=int),
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
            handeye_session_dir_arg,
            handeye_output_dir_arg,
            handeye_board_squares_x_arg,
            handeye_board_squares_y_arg,
            handeye_square_length_m_arg,
            handeye_marker_length_m_arg,
            handeye_aruco_dict_arg,
            handeye_calibration_method_arg,
            handeye_visualize_pause_ms_arg,
            handeye_visualize_arg,
            handeye_visualize_display_scale_arg,
            handeye_ir_clip_max_arg,
            handeye_ir_invert_arg,
            handeye_ir_use_clahe_arg,
            handeye_ir_clahe_clip_arg,
            handeye_ir_clahe_tiles_arg,
            debug_arg,
            # Then include/launch nodes
            ur_driver,
            moveit_stack,
            orbbec_camera,
            rviz_node,
            move_group_demo
        ]
    )
