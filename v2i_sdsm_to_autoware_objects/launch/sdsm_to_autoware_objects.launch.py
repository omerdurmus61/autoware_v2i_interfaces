#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("v2i_sdsm_to_autoware_objects")
    config_file = os.path.join(pkg_share, "config", "sdsm_to_autoware_objects.yaml")

    return LaunchDescription([
        Node(
            package="v2i_sdsm_to_autoware_objects",
            executable="sdsm_to_autoware_objects.py",
            name="sdsm_to_autoware_objects",
            output="screen",
            parameters=[config_file],
        )
    ])