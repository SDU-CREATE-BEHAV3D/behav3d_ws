from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch MoveIt + UR ROS 2 driver for a *real* UR10e by default.

    * Keeps the original defaults (127.0.0.1, mock‑hardware) so nothing breaks
      for simulation users.
    * Exposes two new launch arguments so they can be overridden from CLI:
        • ``robot_ip`` – IP of the physical controller
        • ``use_mock_hardware`` – ``true`` for simulation, ``false`` for real

    Examples
    --------
    ```bash
    # Simulation (old behaviour, no override needed)
    ros2 launch ur_moveit_demos ur10e_moveit_system_live.launch.py

    # Real robot at 192.168.1.130
    ros2 launch ur_moveit_demos ur10e_moveit_system_live.launch.py \
        robot_ip:=192.168.1.130 use_mock_hardware:=false
    ```
    """

    # --------------- Declare overridable arguments -----------------
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

    # ---------------- Paths -----------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("ur_robot_driver"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("ur_moveit_config"), "launch"
    )

    joint_limits_yaml = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "joint_limits.yaml",
    )

    # ---------------- 1) UR driver -----------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # ---------------- 2) Small pause so driver is fully up -----------------
    pause_1s = TimerAction(
        period=1.0,
        actions=[LogInfo(msg="⌛  Driver ready; launching MoveIt …")],
    )

    # ---------------- 3) MoveIt stack -----------------
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_launch_dir, "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "use_sim_time": "false",  # driver already runs in real-time clock
            "joint_limits": joint_limits_yaml,
        }.items(),
    )

    # ---------------- 4) RViz -----------------
    rviz_cfg = os.path.join(
        get_package_share_directory("ur_description"), "rviz", "view_robot.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # ---------------- LaunchDescription -----------------
    return LaunchDescription(
        [
            robot_ip_arg,
            mock_arg,
            ur_driver,
            pause_1s,
            moveit_stack,
            rviz,
        ]
    )

