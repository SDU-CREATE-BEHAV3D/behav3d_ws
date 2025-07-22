from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ur_launch_dir = os.path.join(
        get_package_share_directory("ur_robot_driver"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("ur_moveit_config"), "launch"
    )

    # Path to the joint‑limits YAML that contains the acceleration limits
    joint_limits_yaml = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "joint_limits.yaml",
    )

    # ────────── 1) Driver UR (mock) ──────────
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": "127.0.0.1",
            "use_mock_hardware": "true",
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            # <<< ESTE LAUNCH NO TIENE ARG use_sim_time; el driver ya usa tiempo real
        }.items(),
    )

    # ────────── 2) Pausa breve ──────────
    pause_5s = TimerAction(
        period=1.0,
        actions=[LogInfo(msg="⌛ Driver listo; arrancamos MoveIt…")],
    )

    # ────────── 3) MoveIt ──────────
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "use_sim_time": "false",        #  ⚠️ PASA EL ARGUMENTO A MoveIt
            "joint_limits": joint_limits_yaml,  # <- extra arg so MoveIt loads the YAML with acceleration limits
        }.items(),
    )

    # ────────── 4) RViz ──────────
    rviz_cfg = os.path.join(
        get_package_share_directory("ur_description"),
        "rviz",
        "view_robot.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[{"use_sim_time": False}],   #  ⚠️  Desactiva sim time en RViz
    )

    return LaunchDescription([
        ur_driver,
        pause_5s,
        moveit_stack,
        rviz,
    ])
