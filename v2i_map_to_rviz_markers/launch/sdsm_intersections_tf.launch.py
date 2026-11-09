"""SDSM reference frames in map; calibrated against onboard objects.

See analysis/sdsm_onboard_calibration.md for the bag, method and limits.
Each frame has map as its parent so a calibration cannot move another site.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# x, y, z (metres), yaw (radians). Unobserved sites retain their previous
# cumulative map pose, rather than inheriting corrections from adjacent sites.
MAP_POSES = {
    14867: (-43.788, 9.007, -1.648, 0.00954396),
    40386: (192.940, -100.364, -1.648, 0.0),
    27482: (570.021, -319.318, -1.648, 0.0),
    17342: (-416.642, 167.898, -6.794, 0.00281616),
    24187: (-660.786, 282.143, -13.313, 0.00573253),
    12753: (-786.469, 334.575, -18.163, -0.00011474),
    19846: (-910.687, 390.884, -18.163, 0.0),
    22762: (-1014.314, 391.317, -18.163, 0.0),
    51560: (-1111.071, 388.621, -18.163, 0.0),
    51572: (-1217.344, 387.756, -18.163, 0.0),
    52349: (-1308.253, 424.344, -18.163, 0.0),
}


def generate_launch_description():
    nodes = []
    for intersection_id, (x, y, z, yaw) in MAP_POSES.items():
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"tf_{intersection_id}",
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--roll", "0", "--pitch", "0", "--yaw", str(yaw),
                "--frame-id", "map",
                "--child-frame-id", f"v2i_intersection_{intersection_id}",
            ],
        ))
    return LaunchDescription(nodes)
