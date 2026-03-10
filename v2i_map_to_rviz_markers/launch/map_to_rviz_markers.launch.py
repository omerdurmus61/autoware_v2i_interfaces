#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("v2i_map_to_rviz_markers")

    config_file = os.path.join(
        pkg_share,
        "config",
        "map_to_rviz_markers.yaml"
    )

    return LaunchDescription([
        Node(
            package="v2i_map_to_rviz_markers",
            executable="map_to_rviz_markers.py",
            name="map_to_rviz_markers",
            output="screen",
            parameters=[config_file],
        )
    ])