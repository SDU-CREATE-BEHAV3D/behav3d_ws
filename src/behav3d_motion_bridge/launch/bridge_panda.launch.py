from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build Panda MoveIt parameter set (URDF, SRDF, kinematics, joint_limits, planning, etc.)
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config")
        .to_dict()
    )

    bridge = Node(
        package="behav3d_motion_bridge",
        executable="motion_bridge_node",
        output="screen",
        parameters=[moveit_config, {"use_sim_time": False}],
        # Important so the node accepts parameters it hasn't declared explicitly
        arguments=[],
    )

    return LaunchDescription([bridge])
