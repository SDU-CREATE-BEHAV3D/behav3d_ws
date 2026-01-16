from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='camera')
    return LaunchDescription([
        ns_arg,
        Node(
            package='behav3d_vision',
            executable='rgbd_capture',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[{
                'color_topic': 'color/image_raw',
                'depth_topic': 'depth/image_raw',
                'color_info_topic': 'color/camera_info',
                'depth_info_topic': 'depth/camera_info',
                'save_dir': '/tmp/rgbd',
                'save_every_n': 0,       # 0 = don’t save; set >0 to save every N frames
                'use_approx_sync': True, # safer with real sensors
                'decimate': 1            # 1 = full res
            }]
        ),
    ])
