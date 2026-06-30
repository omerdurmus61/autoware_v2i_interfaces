from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    roi_topic = LaunchConfiguration("roi_topic")
    output_dir = LaunchConfiguration("output_dir")
    save_every_n_frames = LaunchConfiguration("save_every_n_frames")
    min_roi_width = LaunchConfiguration("min_roi_width")
    min_roi_height = LaunchConfiguration("min_roi_height")

    return LaunchDescription(
        [
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
                "output_dir",
                default_value="~/tl_hsv_dataset/roi_crops",
                description="Directory where PNG ROI crops will be saved.",
            ),
            DeclareLaunchArgument(
                "save_every_n_frames",
                default_value="1",
                description="Save crops every N synchronized frames.",
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
            Node(
                package="v2i_spat_camera_fusion",
                executable="traffic_light_roi_cropper_node",
                name="traffic_light_roi_cropper_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "roi_topic": roi_topic,
                        "output_dir": output_dir,
                        "save_every_n_frames": save_every_n_frames,
                        "min_roi_width": min_roi_width,
                        "min_roi_height": min_roi_height,
                    }
                ],
            ),
        ]
    )
