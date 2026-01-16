from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda",
                             package_name="moveit_resources_panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])  # keep it simple: OMPL only
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                # If you have your ompl_only.yaml, include it too:
                # Path inside your package’s share/launch, adjust if different:
                # os.path.join(get_package_share_directory('behav3d_motion_bridge'), 'launch', 'ompl_only.yaml')
            ],
        ),
    ])
