#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_camera_info_topic = LaunchConfiguration("input_camera_info_topic")
    output_camera_info_topic = LaunchConfiguration("output_camera_info_topic")
    target_frame_id = LaunchConfiguration("target_frame_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_camera_info_topic",
            default_value="/sensing/cam2/camera_info",
        ),
        DeclareLaunchArgument(
            "output_camera_info_topic",
            default_value="/sensing/cam2/camera_info_optical",
        ),
        DeclareLaunchArgument(
            "target_frame_id",
            default_value="cam2_calib_optical_link",
        ),
        Node(
            package="v2i_spat_camera_fusion",
            executable="camera_info_frame_republisher_node",
            name="camera_info_frame_republisher",
            output="screen",
            parameters=[{
                "input_camera_info_topic": input_camera_info_topic,
                "output_camera_info_topic": output_camera_info_topic,
                "target_frame_id": target_frame_id,
            }],
        ),
    ])
