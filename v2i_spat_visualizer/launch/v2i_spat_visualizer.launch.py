from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="v2i_spat_visualizer",
                executable="spat_visualizer",
                name="v2i_spat_visualizer",
                output="screen",
                parameters=[
                    {
                        "config_file": PathJoinSubstitution(
                            [FindPackageShare("v2i_spat_visualizer"), "config", "intersections.yaml"]
                        )
                    }
                ],
            )
        ]
    )
