#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("v2i_traffic_light_status_publisher")

    config_file = os.path.join(
        pkg_share,
        "config",
        "v2i_traffic_light_status_publisher.yaml",
    )

    return LaunchDescription([
        Node(
            package="v2i_traffic_light_status_publisher",
            executable="v2i_traffic_light_status_publisher.py",
            name="v2i_traffic_light_status_publisher",
            output="screen",
            parameters=[config_file],
        )
    ])