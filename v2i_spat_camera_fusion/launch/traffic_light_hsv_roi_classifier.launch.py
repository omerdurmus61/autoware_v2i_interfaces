from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    classifier_param_path = LaunchConfiguration("classifier_param_path")
    image_topic = LaunchConfiguration("image_topic")
    roi_topic = LaunchConfiguration("roi_topic")
    output_image_topic = LaunchConfiguration("output_image_topic")
    output_signals_topic = LaunchConfiguration("output_signals_topic")
    traffic_light_id_map_path = LaunchConfiguration("traffic_light_id_map_path")
    min_roi_width = LaunchConfiguration("min_roi_width")
    min_roi_height = LaunchConfiguration("min_roi_height")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "classifier_param_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("v2i_spat_camera_fusion"),
                        "config",
                        "traffic_light_hsv_roi_classifier.param.yaml",
                    ]
                ),
                description="HSV ROI classifier parameter YAML path.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/sensing/cam2/image_raw",
                description="Raw camera image topic.",
            ),
            DeclareLaunchArgument(
                "roi_topic",
                default_value="/traffic_light_map_based_detector/output/rois",
                description="Autoware traffic light ROI array topic.",
            ),
            DeclareLaunchArgument(
                "output_image_topic",
                default_value="/traffic_light_hsv_roi_classifier/debug/image",
                description="Debug image topic with ROI and HSV class labels.",
            ),
            DeclareLaunchArgument(
                "output_signals_topic",
                default_value="/traffic_light_hsv_roi_classifier/output/traffic_signals",
                description="TrafficLightArray output topic.",
            ),
            DeclareLaunchArgument(
                "traffic_light_id_map_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("v2i_spat_camera_fusion"),
                        "config",
                        "traffic_light_id_map.yaml",
                    ]
                ),
                description="Bulbs ID to regularity ID mapping YAML path.",
            ),
            DeclareLaunchArgument(
                "min_roi_width",
                default_value="1",
                description="Minimum clamped ROI width in pixels.",
            ),
            DeclareLaunchArgument(
                "min_roi_height",
                default_value="1",
                description="Minimum clamped ROI height in pixels.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="10.0",
                description="Fixed publish rate for cached traffic light outputs.",
            ),
            Node(
                package="v2i_spat_camera_fusion",
                executable="traffic_light_hsv_roi_classifier_node",
                name="traffic_light_hsv_roi_classifier_node",
                output="screen",
                parameters=[
                    classifier_param_path,
                    {
                        "image_topic": image_topic,
                        "roi_topic": roi_topic,
                        "output_image_topic": output_image_topic,
                        "output_signals_topic": output_signals_topic,
                        "traffic_light_id_map_path": traffic_light_id_map_path,
                        "min_roi_width": min_roi_width,
                        "min_roi_height": min_roi_height,
                        "publish_rate_hz": publish_rate_hz,
                    }
                ],
            ),
        ]
    )
