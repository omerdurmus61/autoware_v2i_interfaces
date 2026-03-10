#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("v2i_map_bridge")

    config_file = os.path.join(
        pkg_share,
        "config",
        "map_udp_bridge.yaml"
    )

    map_node = Node(
        package="v2i_map_bridge",
        executable="map_udp_bridge.py",
        name="map_udp_bridge",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        map_node
    ])