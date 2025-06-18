# File: ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/femto_visualize.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Orbbec camera driver: depth + color
    camera_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera',
        parameters=[{
            'device_vendor_id':   0x2bc5,
            'device_product_id':  0x0669,
            'enable_depth':       True,
            'enable_color':       True,
            'enable_infrared':    False,
            'depth_width':  640, 'depth_height': 576, 'depth_fps': 30,
            'color_width': 1920, 'color_height':1080, 'color_fps': 30,
        }],
        output='screen',
    )

    # 2) Static TF: depth_frame -> color_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_to_color_tf',
        arguments=[
            '0','0','0',        # x y z
            '0','0','0',        # roll pitch yaw
            'camera_depth_frame',
            'camera_color_frame'
        ],
        output='screen',
    )

    # 3) Fused XYZRGB pointcloud node
    pc_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',  # <— corrected name
        name='point_cloud_xyzrgb',
        remappings=[
            ('image_rect',           '/camera/depth/image_raw'),
            ('camera_info',          '/camera/depth/camera_info'),
            ('image_rect_color',     '/camera/color/image_raw'),
            ('camera_info_color',    '/camera/color/camera_info'),
            ('points',               '/point_cloud_xyzrgb/points'),
        ],
        parameters=[{
            'use_color': True
        }],
        output='screen',
    )

    return LaunchDescription([
        camera_node,
        static_tf,
        pc_node,
    ])
