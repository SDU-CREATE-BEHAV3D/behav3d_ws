#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile

def generate_launch_description():
    ur_type               = LaunchConfiguration("ur_type")
    robot_ip              = LaunchConfiguration("robot_ip")
    use_mock_hardware     = LaunchConfiguration("use_mock_hardware")
    launch_rviz           = LaunchConfiguration("launch_rviz")
    enable_femto          = LaunchConfiguration("enable_femto")
    planning_pipeline     = LaunchConfiguration("planning_pipeline")
    group_name            = LaunchConfiguration("group_name")
    eef_link              = LaunchConfiguration("eef_link")
    velocity_scale        = LaunchConfiguration("velocity_scale")
    accel_scale           = LaunchConfiguration("accel_scale")

    workcell_share = FindPackageShare("ur20_workcell")
    moveit_share   = FindPackageShare("ur20_workcell_moveit_config")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
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
            ]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterFile(
            PathJoinSubstitution([moveit_share, "config", "ur20_workcell.srdf"]),
            allow_substs=True,
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": ParameterFile(
            PathJoinSubstitution([moveit_share, "config", "kinematics.yaml"]),
            allow_substs=True,
        )
    }

    planning_params = {
        "move_group": {
            "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
            "planning_pipelines_default": planning_pipeline,
        }
    }

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([workcell_share, "launch", "start_robot.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_share, "launch", "move_group.launch.py"])
        )
    )

    motion_bridge = Node(
        package="behav3d_motion_bridge",
        executable="motion_bridge_node",
        name="motion_bridge_node",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"planning_pipeline": planning_pipeline},
            {"default_group_name": group_name},
            {"default_eef_link": eef_link},
            {"max_velocity_scale": velocity_scale},
            {"max_accel_scale": accel_scale},
            planning_params,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([moveit_share, "config", "move_group.rviz"]),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    declared = [
        DeclareLaunchArgument("ur_type", default_value="ur20"),
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("enable_femto", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("planning_pipeline", default_value="ompl"),
        DeclareLaunchArgument("group_name", default_value="ur_arm"),
        DeclareLaunchArgument("eef_link", default_value="ur20_tool0"),
        DeclareLaunchArgument("velocity_scale", default_value="0.2"),
        DeclareLaunchArgument("accel_scale", default_value="0.2"),
    ]

    return LaunchDescription(declared + [ur_driver, move_group, motion_bridge, rviz])
