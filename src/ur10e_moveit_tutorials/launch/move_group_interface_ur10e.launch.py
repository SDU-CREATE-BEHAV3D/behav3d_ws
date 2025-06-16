from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur10e_moveit_tutorials",
            executable="ur10e_move_group_interface",
            output="screen",
        )
    ])
