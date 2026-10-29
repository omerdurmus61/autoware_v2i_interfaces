#!/usr/bin/env python3

"""Hybrid field-test pipeline with a manually configurable SPaT delay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    delay_ms = LaunchConfiguration("delay_ms")
    delayed_topic = "/v2i/spat/traffic_signals_delayed"
    return LaunchDescription([
        DeclareLaunchArgument("delay_ms", default_value="0.0",
                              description="Added SPaT delay in milliseconds."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("obu_bridge_node", default_value="true"),
        Node(
            package="v2i_spat_camera_fusion",
            executable="spat_delay_node",
            name="spat_delay_node",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "input_topic": "/v2i/spat/traffic_signals",
                "output_topic": delayed_topic,
                "delay_ms": ParameterValue(delay_ms, value_type=float),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("v2i_spat_camera_fusion"),
                "launch", "v2i_spat_camera_fusion.launch.py",
            ])),
            launch_arguments={
                "method": "hybrid",
                "use_sim_time": use_sim_time,
                "obu_bridge_node": LaunchConfiguration("obu_bridge_node"),
                "fusion_spat_input_topic": delayed_topic,
            }.items(),
        ),
    ])
