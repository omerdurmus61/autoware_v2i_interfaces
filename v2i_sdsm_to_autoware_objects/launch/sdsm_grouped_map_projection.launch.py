#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("v2i_sdsm_to_autoware_objects")
    default_config = os.path.join(
        pkg_share,
        "config",
        "sdsm_grouped_map_projection.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the grouped map projection parameter file.",
            ),
            Node(
                package="v2i_sdsm_to_autoware_objects",
                executable="sdsm_grouped_map_projection_node",
                name="sdsm_grouped_map_projection_node",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
