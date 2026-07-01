#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


AUTOWARE_TRAFFIC_SIGNALS_TOPIC = (
    "/perception/traffic_light_recognition/traffic_signals"
)
CAMERA_TRAFFIC_SIGNALS_TOPIC = (
    "/traffic_light_hsv_roi_classifier/output/traffic_signals"
)
SPAT_TRAFFIC_SIGNALS_TOPIC = "/v2i/spat/traffic_signals"


def _method_equals(method: LaunchConfiguration, expected: str) -> IfCondition:
    return IfCondition(PythonExpression(["'", method, "' == '", expected, "'"]))


def generate_launch_description():
    method = LaunchConfiguration("method")

    v2i_spat_bridge_launch = os.path.join(
        get_package_share_directory("v2i_spat_bridge"),
        "launch",
        "spat_udp_bridge.launch.py",
    )
    v2i_tl_status_pkg_share = get_package_share_directory(
        "v2i_traffic_light_status_publisher"
    )
    v2i_tl_status_config = os.path.join(
        v2i_tl_status_pkg_share,
        "config",
        "v2i_traffic_light_status_publisher.yaml",
    )
    camera_info_republisher_launch = os.path.join(
        get_package_share_directory("v2i_spat_camera_fusion"),
        "launch",
        "camera_info_frame_republisher.launch.py",
    )
    hsv_classifier_launch = os.path.join(
        get_package_share_directory("v2i_spat_camera_fusion"),
        "launch",
        "traffic_light_hsv_roi_classifier.launch.py",
    )
    map_based_detector_launch = os.path.join(
        get_package_share_directory("autoware_traffic_light_map_based_detector"),
        "launch",
        "traffic_light_map_based_detector.launch.xml",
    )

    v2i_group = GroupAction(
        condition=_method_equals(method, "v2i"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(v2i_spat_bridge_launch)
            ),
            Node(
                package="v2i_traffic_light_status_publisher",
                executable="v2i_traffic_light_status_publisher.py",
                name="v2i_traffic_light_status_publisher",
                output="screen",
                parameters=[
                    v2i_tl_status_config,
                    {"output_topic": AUTOWARE_TRAFFIC_SIGNALS_TOPIC},
                ],
            ),
        ],
    )

    camera_group = GroupAction(
        condition=_method_equals(method, "camera"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_info_republisher_launch)
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="cam2_link_to_calib_tf",
                arguments=[
                    "0.15",
                    "-0.038",
                    "0.072",
                    "0.06",
                    "-0.14",
                    "0",
                    "vimbax_camera_DEV_000F315E05DE_link",
                    "cam2_calib_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="cam2_calib_to_optical_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "-1.57079632679",
                    "0",
                    "-1.57079632679",
                    "cam2_calib_link",
                    "cam2_calib_optical_link",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(map_based_detector_launch),
                launch_arguments={
                    "input/camera_info": "/sensing/cam2/camera_info_optical",
                    "use_sim_time": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hsv_classifier_launch),
                launch_arguments={
                    "output_signals_topic": AUTOWARE_TRAFFIC_SIGNALS_TOPIC,
                }.items(),
            ),
        ],
    )

    hybrid_group = GroupAction(
        condition=_method_equals(method, "hybrid"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(v2i_spat_bridge_launch)
            ),
            Node(
                package="v2i_traffic_light_status_publisher",
                executable="v2i_traffic_light_status_publisher.py",
                name="v2i_traffic_light_status_publisher",
                output="screen",
                parameters=[
                    v2i_tl_status_config,
                    {"output_topic": SPAT_TRAFFIC_SIGNALS_TOPIC},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_info_republisher_launch)
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="cam2_link_to_calib_tf",
                arguments=[
                    "0.15",
                    "-0.038",
                    "0.072",
                    "0.06",
                    "-0.14",
                    "0",
                    "vimbax_camera_DEV_000F315E05DE_link",
                    "cam2_calib_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="cam2_calib_to_optical_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "-1.57079632679",
                    "0",
                    "-1.57079632679",
                    "cam2_calib_link",
                    "cam2_calib_optical_link",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(map_based_detector_launch),
                launch_arguments={
                    "input/camera_info": "/sensing/cam2/camera_info_optical",
                    "use_sim_time": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hsv_classifier_launch),
                launch_arguments={
                    "output_signals_topic": CAMERA_TRAFFIC_SIGNALS_TOPIC,
                }.items(),
            ),
            Node(
                package="v2i_spat_camera_fusion",
                executable="v2x_spat_fusion_node",
                name="v2x_spat_fusion_node",
                output="screen",
                parameters=[
                    {
                        "camera_input_topic": CAMERA_TRAFFIC_SIGNALS_TOPIC,
                        "spat_input_topic": SPAT_TRAFFIC_SIGNALS_TOPIC,
                        "output_topic": AUTOWARE_TRAFFIC_SIGNALS_TOPIC,
                    }
                ],
            ),
        ],
    )

    invalid_method_warning = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    method,
                    "' != 'v2i' and '",
                    method,
                    "' != 'camera' and '",
                    method,
                    "' != 'hybrid'",
                ]
            )
        ),
        msg=(
            "Invalid 'method' argument. Supported values are: "
            "v2i, camera, hybrid."
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "method",
                default_value="v2i",
                description="Traffic light recognition source: v2i, camera, or hybrid.",
            ),
            v2i_group,
            camera_group,
            hybrid_group,
            invalid_method_warning,
        ]
    )
