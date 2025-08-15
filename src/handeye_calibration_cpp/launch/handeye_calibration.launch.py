

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    session_dir_arg = DeclareLaunchArgument(
        'session_dir',
        default_value='',
        description='Absolute path to a specific session directory. If empty, the node will auto-pick the latest session under output_dir.'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Root directory containing session-* folders (used when session_dir is empty).'
    )

    board_squares_x_arg = DeclareLaunchArgument(
        'board_squares_x',
        default_value='5',
        description='Number of Charuco board squares along X.'
    )

    board_squares_y_arg = DeclareLaunchArgument(
        'board_squares_y',
        default_value='7',
        description='Number of Charuco board squares along Y.'
    )

    square_length_m_arg = DeclareLaunchArgument(
        'square_length_m',
        default_value='0.0288',
        description='Charuco board square length in meters.'
    )

    marker_length_m_arg = DeclareLaunchArgument(
        'marker_length_m',
        default_value='0.022',
        description='Charuco board marker length in meters.'
    )

    aruco_dict_id_arg = DeclareLaunchArgument(
        'aruco_dict_id',
        default_value='0',
        description='OpenCV predefined ArUco dictionary id (e.g., 0..26).'
    )

    calibration_method_arg = DeclareLaunchArgument(
        'calibration_method',
        default_value='tsai',
        description='Hand-eye method: tsai | park | horaud | daniilidis | andreff.'
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Show Charuco detection and axes while processing.'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='rclcpp log level for the node (debug, info, warn, error, fatal).'
    )

    node = Node(
        package='handeye_calibration_cpp',
        executable='standalone',
        name='handeye_calibration',
        output='screen',
        parameters=[{
            'session_dir': LaunchConfiguration('session_dir'),
            'output_dir': LaunchConfiguration('output_dir'),
            'board_squares_x': LaunchConfiguration('board_squares_x'),
            'board_squares_y': LaunchConfiguration('board_squares_y'),
            'square_length_m': LaunchConfiguration('square_length_m'),
            'marker_length_m': LaunchConfiguration('marker_length_m'),
            'aruco_dict_id': LaunchConfiguration('aruco_dict_id'),
            'calibration_method': LaunchConfiguration('calibration_method'),
            'visualize': LaunchConfiguration('visualize'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        session_dir_arg,
        output_dir_arg,
        board_squares_x_arg,
        board_squares_y_arg,
        square_length_m_arg,
        marker_length_m_arg,
        aruco_dict_id_arg,
        calibration_method_arg,
        visualize_arg,
        log_level_arg,
        node
    ])