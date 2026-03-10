#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("v2i_spat_bridge")

    config_file = os.path.join(
        pkg_share,
        "config",
        "spat_udp_bridge.yaml"
    )

    spat_node = Node(
        package="v2i_spat_bridge",
        executable="spat_udp_bridge.py",
        name="spat_udp_bridge",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        spat_node
    ])