from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns = DeclareLaunchArgument('ns', default_value='/camera')
    return LaunchDescription([
        ns,
        Node(
            package='behav3d_vision',
            executable='image_processor',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[{'color_topic': '/camera/color/image_raw'}],
        )
    ])
