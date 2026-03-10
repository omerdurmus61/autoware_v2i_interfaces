#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("v2i_sdsm_bridge")

    config_file = os.path.join(
        pkg_share,
        "config",
        "sdsm_udp_bridge.yaml"
    )

    sdsm_node = Node(
        package="v2i_sdsm_bridge",
        executable="sdsm_udp_bridge",
        name="sdsm_udp_bridge",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        sdsm_node
    ])
