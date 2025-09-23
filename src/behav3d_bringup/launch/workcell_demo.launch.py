from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    enable_femto = LaunchConfiguration("enable_femto")
    rviz_config = LaunchConfiguration("rviz_config")

    # RSP (your workcell xacro → robot_description) via your package’s rsp.launch.py
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur20_workcell"), "launch", "rsp.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "mock_sensor_commands": "false",
            "headless_mode": "false",
            "enable_femto": enable_femto,
        }.items(),
    )

    # UR driver (mock or real)
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "description_launchfile": PathJoinSubstitution([FindPackageShare("ur20_workcell"), "launch", "rsp.launch.py"]),
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "tf_prefix": ur_type,  # matches your prefixed workcell
            "launch_rviz": "false",
        }.items(),
    )

    # MoveIt (workcell config)
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur20_workcell_moveit_config"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            # ensure these two are published for other nodes
            "publish_robot_description": "true",
            "publish_robot_description_semantic": "true",
            # pass through UR args if your move_group launch consumes them
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    # RViz with MoveIt MotionPlanning panel
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # Motion bridge service
    bridge = Node(
        package="behav3d_motion_bridge",
        executable="motion_bridge_node",
        name="motion_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur20"),
        DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("enable_femto", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur20_workcell_moveit_config"),
                "config", "moveit.rviz"  # adjust if your file name differs
            ])
        ),
        rsp_launch,
        ur_control,
        moveit,
        bridge,
        rviz,
    ])
