#!/usr/bin/env python3

# Test commands:
#   ros2 topic echo /raptor_dbw_interface/driver_input_report
#   ros2 topic echo /awapi/tmp/virtual_traffic_light_states
#   ros2 launch v2i_driver_approved_virtual_traffic_light driver_approved_virtual_traffic_light.launch.py
#
# Safety note:
#   This node only publishes virtual traffic light approval state.
#   It must not send throttle, brake, steering, or any DBW command.
#   Final go/no-go behavior remains on the Autoware planning side.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "v2i_driver_approved_virtual_traffic_light"
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(
        package_share_directory,
        "config",
        "driver_approved_virtual_traffic_light.param.yaml",
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable="driver_approved_virtual_traffic_light_node",
            name="driver_approved_virtual_traffic_light_node",
            output="screen",
            parameters=[config_file],
        )
    ])
