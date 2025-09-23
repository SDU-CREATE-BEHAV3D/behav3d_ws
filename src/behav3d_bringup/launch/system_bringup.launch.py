from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- UR20 bringup ---
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('behav3d_bringup'),
                         'launch', 'kinematics_ur20_demo.launch.py')),
        launch_arguments={
            'robot_ip': '0.0.0.0',
            'use_mock_hardware': 'true',
            'group': 'ur_arm',
            'root_link': 'world',
            'eef_link': 'ur20_tool0',

        }.items()
    )

    # --- MoveIt bridge ---
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('behav3d_motion_bridge'),
                         'launch', 'bridge_workcell.launch.py')),
        launch_arguments={
            'exec_mode': 'move_group',
            'viz_enabled': 'true',
        }.items()
    )

    # --- Orbbec camera (FORCE namespace and name) ---
    cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'),
                        'launch', 'femto_mega.launch.py')),
        launch_arguments={
            'namespace': '/camera',
            'camera_name': 'camera',

            # cut the load:
            'enable_point_cloud': 'false',   # turn OFF for now
            'depth_registration': 'true',
            'enable_d2c_viewer': 'false',
            'color_fps': '15',
            'depth_fps': '15',
            'color_width': '640', 'color_height': '480',
            'depth_width': '640', 'depth_height': '480',
        }.items()
    )


    # --- Vision capture node (start a moment after camera) ---
    imgproc = Node(
        package='behav3d_vision',
        executable='image_processor',
        namespace='/camera',
        name='image_processor',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[{
            # absolute topics to avoid double namespacing surprises
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'default_save_dir': '/home/create_ros/captures',
        }],
    )

    # start image processor 2s after camera include (avoids “waiting for topics” noise)
    delayed_imgproc = TimerAction(period=2.0, actions=[imgproc])

    return LaunchDescription([
        bringup,
        bridge,
        cam,
        delayed_imgproc,
    ])
