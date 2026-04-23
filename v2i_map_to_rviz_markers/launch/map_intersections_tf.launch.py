from launch import LaunchDescription
from launch_ros.actions import Node


def static_tf(x, y, z, roll, pitch, yaw, parent, child, name):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            str(x),
            str(y),
            str(z),
            str(roll),
            str(pitch),
            str(yaw),
            parent,
            child,
        ],
    )


def generate_launch_description():
    nodes = []

    nodes.append(static_tf(0, 0, 0, 0, 0, 0,
                           "map", "v2i_intersection_14867", "tf_14867"))

    nodes.append(static_tf(237.329, -108.967, 0, 0, 0, 0,
                           "v2i_intersection_14867", "v2i_intersection_40386", "tf_40386"))

    nodes.append(static_tf(377.081, -218.954, 0, 0, 0, 0,
                           "v2i_intersection_40386", "v2i_intersection_27482", "tf_27482"))

    nodes.append(static_tf(-371.567, 160.854, 0, 0, 0, 0,
                           "v2i_intersection_14867", "v2i_intersection_17342", "tf_17342"))

    nodes.append(static_tf(-243.188, 111.707, 0, 0, 0, 0,
                           "v2i_intersection_17342", "v2i_intersection_24187", "tf_24187"))

    nodes.append(static_tf(-121.803, 57.102, 0, 0, 0, 0,
                           "v2i_intersection_24187", "v2i_intersection_12753", "tf_12753"))

    nodes.append(static_tf(-127.057, 55.681, 0, 0, 0, 0,
                           "v2i_intersection_12753", "v2i_intersection_19846", "tf_19846"))

    nodes.append(static_tf(-103.627, 0.433, 0, 0, 0, 0,
                           "v2i_intersection_19846", "v2i_intersection_22762", "tf_22762"))

    nodes.append(static_tf(-96.757, -2.696, 0, 0, 0, 0,
                           "v2i_intersection_22762", "v2i_intersection_51560", "tf_51560"))

    nodes.append(static_tf(-106.273, -0.865, 0, 0, 0, 0,
                           "v2i_intersection_51560", "v2i_intersection_51572", "tf_51572"))

    nodes.append(static_tf(-90.909, 36.588, 0, 0, 0, 0,
                           "v2i_intersection_51572", "v2i_intersection_52349", "tf_52349"))

    return LaunchDescription(nodes)