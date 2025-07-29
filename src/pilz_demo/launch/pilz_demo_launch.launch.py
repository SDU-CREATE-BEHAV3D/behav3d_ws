#!/usr/bin/env python3
          
# =============================================================================
#   ____  _____ _   _    ___     _______ ____  
#  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
#  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
#  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
#  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
#                                               
#                                               
# Author: Lucas José Helle <luh@iti.sdu.dk>
# Maintainers:
#   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
#   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
# Institute: University of Southern Denmark (Syddansk Universitet)
# Date: 2025-07
# =============================================================================

from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Launch UR driver + MoveIt stack for a UR10e *and* a MoveIt‑Py helper node.

    - Keeps original behaviour (simulation by default, real robot via overrides).
    - Re‑creates the same MoveItConfigsBuilder used inside *ur_moveit.launch.py*
      so that the Python helper receives an *identical* parameter set.
    """

    # -------------------------------------------------------------------------
    # 1) User‑overridable CLI arguments
    # -------------------------------------------------------------------------
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1",
        description="IP address of the UR controller (real robot).",
    )
    mock_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="true = simulation/mock, false = real hardware",
    )

    # -------------------------------------------------------------------------
    # 2) Common paths
    # -------------------------------------------------------------------------
    ur_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell"), "launch"
    )
    moveit_launch_dir = os.path.join(
        get_package_share_directory("i40_workcell_moveit_config"), "launch"
    )
    
    # -------------------------------------------------------------------------
    # 3) UR driver (real robot or mock) Calling I40_workcell start_robot launch
    # -------------------------------------------------------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, "start_robot.launch.py")),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # 4) MoveIt stack (Initialize I40_workspace_moveit_config movegroup)
    # -------------------------------------------------------------------------

    
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_launch_dir, "move_group.launch.py")),
    )

    # -------------------------------------------------------------------------
    # 5) Re‑build the *same* MoveIt config so we can share it with a helper node
    # -------------------------------------------------------------------------

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="i40_workcell_moveit_config")
        .robot_description_semantic(Path("config") / "ur.srdf")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("pilz_demo"),
                "config/pilz_demo.yaml",
            )
        )
        .to_moveit_configs()
    )

    # Helper node that runs any MoveIt‑Py script (default: tutorial)
    moveit_py_node = Node(
        package="pilz_demo",  #  👉 replace with your own package if needed
        executable="run_demo",
        name="pilz_demo_moveit_py",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    # -------------------------------------------------------------------------
    # 6) Assemble LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription(
        [
            robot_ip_arg,
            mock_arg,
            ur_driver,
            moveit_stack,
            moveit_py_node,

        ]
    )

    # # -------------------------------------------------------------------------
    # # Old) RViz (optional visualisation)
    # # -------------------------------------------------------------------------
    # rviz_cfg = os.path.join(
    #     get_package_share_directory("ur_description"), "rviz", "view_robot.rviz"
    # )
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz_cfg],
    #     output="screen",
    #     parameters=[{"use_sim_time": False}],
    # )
