from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Only build descriptions (no extra pipeline config from the package)
    mcfg = MoveItConfigsBuilder(
        "panda", package_name="moveit_resources_panda_moveit_config"
    ).to_moveit_configs()

    # Inline OMPL config, applied LAST
    ompl_inline = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "start_state_max_bounds_error": 0.1,
        },
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            # descriptions ONLY
            mcfg.robot_description,
            mcfg.robot_description_semantic,
            mcfg.robot_description_kinematics,
            # minimal, explicit OMPL config
            ompl_inline,
            {"use_sim_time": False},
        ],
    )
    return LaunchDescription([move_group])
