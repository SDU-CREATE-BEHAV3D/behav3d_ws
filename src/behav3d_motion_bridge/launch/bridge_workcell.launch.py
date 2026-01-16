#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # args
    ur_type           = LaunchConfiguration("ur_type")
    robot_ip          = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_femto      = LaunchConfiguration("enable_femto")

    workcell_share = FindPackageShare("ur20_workcell")
    moveit_pkg     = "ur20_workcell_moveit_config"

    # --- Build robot_description via xacro command (STRING, not Path/ParameterFile) ---
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([workcell_share, "urdf", "ur20_bolt_controlled.urdf.xacro"]), " ",
            "robot_ip:=", robot_ip, " ",
            "ur_type:=", ur_type, " ",
            "kinematics_parameters_file:=",
                PathJoinSubstitution([workcell_share, "config", "my_robot_calibration.yaml"]), " ",
            "use_mock_hardware:=", use_mock_hardware, " ",
            "mock_sensor_commands:=false ",
            "headless_mode:=false ",
            "enable_femto:=", enable_femto,
        ])
    }

    # --- Everything else from the MoveIt config package ---
    mcfg = (
        MoveItConfigsBuilder(robot_name="ur", package_name=moveit_pkg)
        # DO NOT call .robot_description(...) here
        .to_moveit_configs()
    )

    bridge = Node(
        package="behav3d_motion_bridge",
        executable="motion_bridge_node",
        name="motion_bridge_node",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            robot_description,                           # our xacro string
            mcfg.robot_description_semantic,             # SRDF
            mcfg.robot_description_kinematics,           # kinematics.yaml
            mcfg.planning_pipelines,                     # pipelines (ompl/pilz/etc.)
            mcfg.joint_limits,                           # joint_limits.yaml
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur20"),
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("enable_femto", default_value="false"),
        bridge,
    ])
