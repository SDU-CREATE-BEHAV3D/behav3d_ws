from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Optional (kept for flexibility)
    mesh_dir_arg = DeclareLaunchArgument(
        'mesh_dir',
        default_value='/home/lab/robot/meshes',
        description='Directory containing reconstructed mesh files'
    )

    # Hardcoded to the correct robot frame
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='ur20_base_link',
        description='Reference frame for mesh visualization'
    )

    visualizer_node = Node(
        package='world_visualizer',
        executable='mesh_visualizer',
        name='mesh_visualizer',
        output='screen',
        parameters=[
            {'mesh_dir': LaunchConfiguration('mesh_dir')},
            {'frame_id': LaunchConfiguration('frame_id')}
        ]
    )

    return LaunchDescription([
        mesh_dir_arg,
        frame_id_arg,
        visualizer_node
    ])
