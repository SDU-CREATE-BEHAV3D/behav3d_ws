#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_color_to_depth_arg = DeclareLaunchArgument(
        "enable_color_to_depth",
        default_value="true",
        description="Start /reconstruct/color_to_depth service node",
    )
    enable_tsdf_cropped_arg = DeclareLaunchArgument(
        "enable_tsdf_cropped",
        default_value="true",
        description="Start /reconstruct/tsdf_cropped service node",
    )
    enable_tsdf_object_extract_arg = DeclareLaunchArgument(
        "enable_tsdf_object_extract",
        default_value="true",
        description="Start /reconstruct/tsdf_object_extract service node",
    )

    scan_folder_arg = DeclareLaunchArgument(
        "scan_folder",
        default_value="manual_caps",
        description="Default scan folder used when request scan_folder is empty",
    )
    visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value="false",
        description="Default visualization flag for reconstruction services",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="CPU:0",
        description="Default Open3D device for TSDF services",
    )

    color_to_depth_node = Node(
        package="behav3d_sense",
        executable="color_to_depth_service",
        name="color_to_depth_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_color_to_depth")),
        parameters=[
            {
                "scan_folder": LaunchConfiguration("scan_folder"),
                "visualize": ParameterValue(LaunchConfiguration("visualize"), value_type=bool),
            }
        ],
    )

    tsdf_cropped_node = Node(
        package="behav3d_sense",
        executable="tsdf_cropped_service",
        name="tsdf_cropped_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_tsdf_cropped")),
        parameters=[
            {
                "scan_folder": LaunchConfiguration("scan_folder"),
                "visualize": ParameterValue(LaunchConfiguration("visualize"), value_type=bool),
                "device": LaunchConfiguration("device"),
            }
        ],
    )

    tsdf_object_extract_node = Node(
        package="behav3d_sense",
        executable="tsdf_object_extract_service",
        name="tsdf_object_extract_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_tsdf_object_extract")),
        parameters=[
            {
                "scan_folder": LaunchConfiguration("scan_folder"),
                "visualize": ParameterValue(LaunchConfiguration("visualize"), value_type=bool),
                "device": LaunchConfiguration("device"),
            }
        ],
    )

    return LaunchDescription(
        [
            enable_color_to_depth_arg,
            enable_tsdf_cropped_arg,
            enable_tsdf_object_extract_arg,
            scan_folder_arg,
            visualize_arg,
            device_arg,
            color_to_depth_node,
            tsdf_cropped_node,
            tsdf_object_extract_node,
        ]
    )
