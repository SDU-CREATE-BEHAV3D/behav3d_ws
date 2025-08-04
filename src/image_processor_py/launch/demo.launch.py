#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the orbbec_camera package to include its Femto Mega launch
    orbbec_pkg = get_package_share_directory('orbbec_camera')

    femto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_pkg, 'launch', 'femto_mega.launch.py')
        ),
        # you can override any launch args the Femto file supports here:
        # launch_arguments={'enumerate_net_device': 'true'}.items()
    )

    # Your image processor node
    img_proc_node = Node(
        package='image_processor_py',
        executable='image_processor_capture',
        name='image_processor_capture',
        output='screen',
        parameters=[{
            # match these to your setup or override on the CLI
            'output_dir': os.path.expanduser('~/captures'),
            'base_frame': 'ur10e_wrist_3_link',
            'ee_frame': 'femto__depth_optical_frame',
        }],
        # remap if your topics differ:
        # remappings=[
        #     ('/camera/color/image_raw', '/camera/color/image_raw'),
        #     ('/camera/depth/image_raw', '/camera/depth/image_raw'),
        # ],
    )

    return LaunchDescription([
        femto_launch,
        img_proc_node,
    ])
