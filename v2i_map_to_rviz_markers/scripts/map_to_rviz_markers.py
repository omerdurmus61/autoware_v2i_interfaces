#!/usr/bin/env python3

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from v2i_map_msgs.msg import MapData, MapIntersection, MapLane


class MapToRvizMarkers(Node):
    def __init__(self) -> None:
        super().__init__("map_to_rviz_markers")

        self.declare_parameter("input_topic", "/v2i/map/raw")
        self.declare_parameter("output_topic", "/v2i/map/lane_markers")
        self.declare_parameter("frame_namespace", "v2i_intersection_")

        # If true, publish all intersections found in the message.
        # If false, only publish target_intersection_id.
        self.declare_parameter("publish_all_intersections", True)
        self.declare_parameter("target_intersection_id", 14867)

        self.declare_parameter("node_scale_x", 0.01)
        self.declare_parameter("node_scale_y", 0.01)
        self.declare_parameter("node_offset_x", 0.0)
        self.declare_parameter("node_offset_y", 0.0)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("line_z", 0.05)
        self.declare_parameter("line_width", 0.15)

        self.declare_parameter("publish_lane_ids", True)
        self.declare_parameter("lane_id_z", 0.5)
        self.declare_parameter("lane_id_scale", 1.0)

        self.declare_parameter("publish_connections", True)
        self.declare_parameter("connection_z", 0.1)
        self.declare_parameter("connection_width", 0.08)

        self.declare_parameter("publish_direction_arrows", True)
        self.declare_parameter("arrow_z", 0.12)
        self.declare_parameter("arrow_scale_x", 0.8)
        self.declare_parameter("arrow_scale_y", 0.2)
        self.declare_parameter("arrow_scale_z", 0.2)

        self.declare_parameter("publish_signal_group_labels", True)
        self.declare_parameter("signal_group_z", 0.35)
        self.declare_parameter("signal_group_scale", 0.7)

        self.declare_parameter("lane_rgba", [0.0, 1.0, 0.0, 1.0])
        self.declare_parameter("connection_rgba", [1.0, 0.6, 0.0, 1.0])
        self.declare_parameter("lane_id_rgba", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("arrow_rgba", [0.0, 0.8, 1.0, 1.0])
        self.declare_parameter("signal_group_rgba", [1.0, 0.2, 0.2, 1.0])

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.frame_namespace = str(self.get_parameter("frame_namespace").value)
        self.publish_all_intersections = bool(self.get_parameter("publish_all_intersections").value)
        self.target_intersection_id = int(self.get_parameter("target_intersection_id").value)

        self.node_scale_x = float(self.get_parameter("node_scale_x").value)
        self.node_scale_y = float(self.get_parameter("node_scale_y").value)
        self.node_offset_x = float(self.get_parameter("node_offset_x").value)
        self.node_offset_y = float(self.get_parameter("node_offset_y").value)
        self.swap_xy = bool(self.get_parameter("swap_xy").value)
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.line_z = float(self.get_parameter("line_z").value)
        self.line_width = float(self.get_parameter("line_width").value)

        self.publish_lane_ids = bool(self.get_parameter("publish_lane_ids").value)
        self.lane_id_z = float(self.get_parameter("lane_id_z").value)
        self.lane_id_scale = float(self.get_parameter("lane_id_scale").value)

        self.publish_connections = bool(self.get_parameter("publish_connections").value)
        self.connection_z = float(self.get_parameter("connection_z").value)
        self.connection_width = float(self.get_parameter("connection_width").value)

        self.publish_direction_arrows = bool(self.get_parameter("publish_direction_arrows").value)
        self.arrow_z = float(self.get_parameter("arrow_z").value)
        self.arrow_scale_x = float(self.get_parameter("arrow_scale_x").value)
        self.arrow_scale_y = float(self.get_parameter("arrow_scale_y").value)
        self.arrow_scale_z = float(self.get_parameter("arrow_scale_z").value)

        self.publish_signal_group_labels = bool(self.get_parameter("publish_signal_group_labels").value)
        self.signal_group_z = float(self.get_parameter("signal_group_z").value)
        self.signal_group_scale = float(self.get_parameter("signal_group_scale").value)

        self.lane_rgba = self._read_rgba_param("lane_rgba")
        self.connection_rgba = self._read_rgba_param("connection_rgba")
        self.lane_id_rgba = self._read_rgba_param("lane_id_rgba")
        self.arrow_rgba = self._read_rgba_param("arrow_rgba")
        self.signal_group_rgba = self._read_rgba_param("signal_group_rgba")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            MapData,
            input_topic,
            self.map_callback,
            qos,
        )

        self.pub = self.create_publisher(
            MarkerArray,
            output_topic,
            10,
        )

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing MarkerArray on: {output_topic}")
        self.get_logger().info(f"Frame namespace: {self.frame_namespace}")
        self.get_logger().info(f"Publish all intersections: {self.publish_all_intersections}")
        if not self.publish_all_intersections:
            self.get_logger().info(f"Target intersection_id: {self.target_intersection_id}")

    def _read_rgba_param(self, name: str) -> Tuple[float, float, float, float]:
        values = self.get_parameter(name).value
        if not isinstance(values, list) or len(values) != 4:
            self.get_logger().warn(f"Parameter '{name}' must be a list of 4 floats. Using fallback.")
            return 1.0, 1.0, 1.0, 1.0
        return float(values[0]), float(values[1]), float(values[2]), float(values[3])

    def _frame_id_for_intersection(self, intersection_id: int) -> str:
        return f"{self.frame_namespace}{intersection_id}"

    def _make_header(self, stamp, frame_id: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def _convert_xy(self, raw_x: int, raw_y: int) -> Tuple[float, float]:
        x = raw_x * self.node_scale_x + self.node_offset_x
        y = raw_y * self.node_scale_y + self.node_offset_y

        if self.swap_xy:
            x, y = y, x
        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y

        return x, y

    def _selected_intersections(self, msg: MapData) -> List[MapIntersection]:
        if self.publish_all_intersections:
            return list(msg.intersections)

        selected: List[MapIntersection] = []
        for intersection in msg.intersections:
            if intersection.id.id == self.target_intersection_id:
                selected.append(intersection)
                break
        return selected

    def _lane_points(self, lane: MapLane, z_value: float) -> List[Point]:
        points: List[Point] = []
        for node in lane.nodes:
            x, y = self._convert_xy(node.x, node.y)
            p = Point()
            p.x = x
            p.y = y
            p.z = z_value
            points.append(p)
        return points

    def _lane_midpoint(self, lane: MapLane, z_value: float) -> Point:
        points = self._lane_points(lane, z_value)
        midpoint = Point()

        if not points:
            midpoint.x = 0.0
            midpoint.y = 0.0
            midpoint.z = z_value
            return midpoint

        return points[len(points) // 2]

    def _make_lane_line_marker(
        self,
        lane: MapLane,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Marker:
        marker = Marker()
        marker.header = self._make_header(stamp, frame_id)
        marker.ns = f"lane_centerlines_{namespace_suffix}"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.line_width

        r, g, b, a = self.lane_rgba
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.points = self._lane_points(lane, self.line_z)
        marker.pose.orientation.w = 1.0
        return marker

    def _make_lane_text_marker(
        self,
        lane: MapLane,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Marker:
        marker = Marker()
        marker.header = self._make_header(stamp, frame_id)
        marker.ns = f"lane_ids_{namespace_suffix}"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = self._lane_midpoint(lane, self.lane_id_z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.lane_id_scale

        r, g, b, a = self.lane_id_rgba
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.text = str(lane.lane_id)
        return marker

    def _make_arrow_marker(
        self,
        lane: MapLane,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Optional[Marker]:
        points = self._lane_points(lane, self.arrow_z)
        if len(points) < 2:
            return None

        marker = Marker()
        marker.header = self._make_header(stamp, frame_id)
        marker.ns = f"lane_direction_arrows_{namespace_suffix}"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = self.arrow_scale_x
        marker.scale.y = self.arrow_scale_y
        marker.scale.z = self.arrow_scale_z

        r, g, b, a = self.arrow_rgba
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.points = [points[-2], points[-1]]
        marker.pose.orientation.w = 1.0
        return marker

    def _make_connection_marker(
        self,
        from_lane: MapLane,
        to_lane: MapLane,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Optional[Marker]:
        from_points = self._lane_points(from_lane, self.connection_z)
        to_points = self._lane_points(to_lane, self.connection_z)

        if not from_points or not to_points:
            return None

        marker = Marker()
        marker.header = self._make_header(stamp, frame_id)
        marker.ns = f"lane_connections_{namespace_suffix}"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.connection_width

        r, g, b, a = self.connection_rgba
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.points = [from_points[-1], to_points[0]]
        marker.pose.orientation.w = 1.0
        return marker

    def _make_signal_group_text_marker(
        self,
        from_lane: MapLane,
        to_lane: MapLane,
        signal_group: int,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Optional[Marker]:
        from_points = self._lane_points(from_lane, self.signal_group_z)
        to_points = self._lane_points(to_lane, self.signal_group_z)

        if not from_points or not to_points:
            return None

        start_pt = from_points[-1]
        end_pt = to_points[0]

        marker = Marker()
        marker.header = self._make_header(stamp, frame_id)
        marker.ns = f"signal_group_labels_{namespace_suffix}"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.5 * (start_pt.x + end_pt.x)
        marker.pose.position.y = 0.5 * (start_pt.y + end_pt.y)
        marker.pose.position.z = self.signal_group_z
        marker.pose.orientation.w = 1.0

        marker.scale.z = self.signal_group_scale

        r, g, b, a = self.signal_group_rgba
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        marker.text = f"SG:{signal_group}"
        return marker

    def map_callback(self, msg: MapData) -> None:
        intersections = self._selected_intersections(msg)
        if not intersections:
            if self.publish_all_intersections:
                self.get_logger().warn("No intersections found in MAP message.")
            else:
                self.get_logger().warn(
                    f"Target intersection_id={self.target_intersection_id} not found in message."
                )
            return

        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_id = 0

        for intersection in intersections:
            intersection_id = int(intersection.id.id)
            frame_id = self._frame_id_for_intersection(intersection_id)
            namespace_suffix = str(intersection_id)

            lane_lookup: Dict[int, MapLane] = {
                int(lane.lane_id): lane for lane in intersection.lane_set
            }

            for lane in intersection.lane_set:
                marker_array.markers.append(
                    self._make_lane_line_marker(
                        lane=lane,
                        stamp=stamp,
                        marker_id=marker_id,
                        frame_id=frame_id,
                        namespace_suffix=namespace_suffix,
                    )
                )
                marker_id += 1

                if self.publish_lane_ids:
                    marker_array.markers.append(
                        self._make_lane_text_marker(
                            lane=lane,
                            stamp=stamp,
                            marker_id=marker_id,
                            frame_id=frame_id,
                            namespace_suffix=namespace_suffix,
                        )
                    )
                    marker_id += 1

                if self.publish_direction_arrows:
                    arrow_marker = self._make_arrow_marker(
                        lane=lane,
                        stamp=stamp,
                        marker_id=marker_id,
                        frame_id=frame_id,
                        namespace_suffix=namespace_suffix,
                    )
                    if arrow_marker is not None:
                        marker_array.markers.append(arrow_marker)
                        marker_id += 1

            if self.publish_connections or self.publish_signal_group_labels:
                for lane in intersection.lane_set:
                    for connection in lane.connections:
                        to_lane = lane_lookup.get(int(connection.connecting_lane))
                        if to_lane is None:
                            continue

                        if self.publish_connections:
                            conn_marker = self._make_connection_marker(
                                from_lane=lane,
                                to_lane=to_lane,
                                stamp=stamp,
                                marker_id=marker_id,
                                frame_id=frame_id,
                                namespace_suffix=namespace_suffix,
                            )
                            if conn_marker is not None:
                                marker_array.markers.append(conn_marker)
                                marker_id += 1

                        if self.publish_signal_group_labels:
                            sg_marker = self._make_signal_group_text_marker(
                                from_lane=lane,
                                to_lane=to_lane,
                                signal_group=int(connection.signal_group),
                                stamp=stamp,
                                marker_id=marker_id,
                                frame_id=frame_id,
                                namespace_suffix=namespace_suffix,
                            )
                            if sg_marker is not None:
                                marker_array.markers.append(sg_marker)
                                marker_id += 1

        self.pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapToRvizMarkers()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()