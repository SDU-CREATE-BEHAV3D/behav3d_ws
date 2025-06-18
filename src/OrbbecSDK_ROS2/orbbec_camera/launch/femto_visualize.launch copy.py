from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Orbbec camera driver (depth + color)
    camera_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='orbbec_camera',
        parameters=[{
            'device_vendor_id':  0x2bc5,
            'device_product_id': 0x0669,
            'enable_depth':      True,
            'enable_color':      True,
            'enable_infrared':   False,
            'depth_width':  640, 'depth_height': 576, 'depth_fps': 30,
            'color_width': 1920, 'color_height':1080, 'color_fps': 30,
        }],
        output='screen',
    )

    # 2) Static TF from depth_frame → color_frame (so DepthCloud can work in color frame)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_to_color_tf',
        arguments=[
            '0','0','0',       # x y z
            '0','0','0',       # roll pitch yaw
            'camera_depth_frame',
            'camera_color_frame'
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_node,
        static_tf,
    ])
