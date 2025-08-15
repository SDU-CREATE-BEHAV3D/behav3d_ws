

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    session_dir_arg = DeclareLaunchArgument(
        'handeye_session_dir',
        default_value='',
        description='Absolute path to a specific hand-eye session directory. If empty, auto-pick latest under handeye_output_dir.'
    )

    output_dir_arg = DeclareLaunchArgument(
        'handeye_output_dir',
        default_value='~/behav3d_ws/captures',
        description='Root directory containing session-* folders (used when handeye_session_dir is empty).'
    )

    board_squares_x_arg = DeclareLaunchArgument(
        'handeye_board_squares_x',
        default_value='12',
        description='Number of Charuco board squares along X.'
    )

    board_squares_y_arg = DeclareLaunchArgument(
        'handeye_board_squares_y',
        default_value='9',
        description='Number of Charuco board squares along Y.'
    )

    square_length_m_arg = DeclareLaunchArgument(
        'handeye_square_length_m',
        default_value='0.03',
        description='Charuco board square length in meters.'
    )

    marker_length_m_arg = DeclareLaunchArgument(
        'handeye_marker_length_m',
        default_value='0.022',
        description='Charuco board marker length in meters.'
    )

    aruco_dict_arg = DeclareLaunchArgument(
        'handeye_aruco_dict',
        default_value='DICT_5X5_100',
        description='ArUco dictionary name (e.g., "DICT_5X5_100").'
    )

    calibration_method_arg = DeclareLaunchArgument(
        'handeye_calibration_method',
        default_value='tsai',
        description='Hand-eye method: tsai | park | horaud | daniilidis | andreff.'
    )

    visualize_arg = DeclareLaunchArgument(
        'handeye_visualize',
        default_value='true',
        description='Show Charuco detection and axes while processing.'
    )

    visualize_pause_ms_arg = DeclareLaunchArgument(
        'handeye_visualize_pause_ms',
        default_value='1000',
        description='Wait time for visualization window in milliseconds.'
    )

    visualize_display_scale_arg = DeclareLaunchArgument(
        'handeye_visualize_display_scale',
        default_value='0.5',
        description='Uniform scale factor for on-screen visualization (0.1–4.0).'
    )

    # IR preprocessing args
    ir_clip_max_arg = DeclareLaunchArgument(
        'handeye_ir_clip_max',
        default_value='-1',
        description='If >0, clamp 16-bit IR to this value before normalization.'
    )
    ir_invert_arg = DeclareLaunchArgument(
        'handeye_ir_invert',
        default_value='false',
        description='Invert IR image after normalization.'
    )
    ir_use_clahe_arg = DeclareLaunchArgument(
        'handeye_ir_use_clahe',
        default_value='true',
        description='Apply CLAHE to IR image.'
    )
    ir_clahe_clip_arg = DeclareLaunchArgument(
        'handeye_ir_clahe_clip',
        default_value='3.0',
        description='CLAHE clip limit for IR.'
    )
    ir_clahe_tiles_arg = DeclareLaunchArgument(
        'handeye_ir_clahe_tiles',
        default_value='8',
        description='CLAHE tile grid size (NxN) for IR.'
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
            'handeye_session_dir': LaunchConfiguration('handeye_session_dir'),
            'handeye_output_dir': LaunchConfiguration('handeye_output_dir'),
            'handeye_board_squares_x': LaunchConfiguration('handeye_board_squares_x'),
            'handeye_board_squares_y': LaunchConfiguration('handeye_board_squares_y'),
            'handeye_square_length_m': LaunchConfiguration('handeye_square_length_m'),
            'handeye_marker_length_m': LaunchConfiguration('handeye_marker_length_m'),
            'handeye_aruco_dict': LaunchConfiguration('handeye_aruco_dict'),
            'handeye_calibration_method': LaunchConfiguration('handeye_calibration_method'),
            'handeye_visualize': LaunchConfiguration('handeye_visualize'),
            'handeye_visualize_pause_ms': LaunchConfiguration('handeye_visualize_pause_ms'),
            'handeye_visualize_display_scale': LaunchConfiguration('handeye_visualize_display_scale'),
            'handeye_ir_clip_max': LaunchConfiguration('handeye_ir_clip_max'),
            'handeye_ir_invert': LaunchConfiguration('handeye_ir_invert'),
            'handeye_ir_use_clahe': LaunchConfiguration('handeye_ir_use_clahe'),
            'handeye_ir_clahe_clip': LaunchConfiguration('handeye_ir_clahe_clip'),
            'handeye_ir_clahe_tiles': LaunchConfiguration('handeye_ir_clahe_tiles'),
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
        aruco_dict_arg,
        calibration_method_arg,
        visualize_arg,
        visualize_pause_ms_arg,
        visualize_display_scale_arg,
        ir_clip_max_arg,
        ir_invert_arg,
        ir_use_clahe_arg,
        ir_clahe_clip_arg,
        ir_clahe_tiles_arg,
        log_level_arg,
        node
    ])