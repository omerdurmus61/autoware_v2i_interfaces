#!/usr/bin/env python3

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from v2i_map_msgs.msg import MapData, MapIntersection, MapLane, MapNode


class MapToRvizMarkers(Node):
    def __init__(self) -> None:
        super().__init__("map_to_rviz_markers")

        self.declare_parameter("input_topic", "/v2i/map/raw")
        self.declare_parameter("output_topic", "/v2i/map/lane_markers")
        self.declare_parameter("frame_namespace", "v2i_intersection_")

        self.declare_parameter("publish_all_intersections", True)
        self.declare_parameter("target_intersection_id", 14867)

        # Additional global scaling if needed after node-type scaling
        self.declare_parameter("node_scale_x", 1.0)
        self.declare_parameter("node_scale_y", 1.0)
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

        self.declare_parameter("debug_log_lanes", False)

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

        self.lane_rgba = tuple(float(v) for v in self.get_parameter("lane_rgba").value)
        self.connection_rgba = tuple(float(v) for v in self.get_parameter("connection_rgba").value)
        self.lane_id_rgba = tuple(float(v) for v in self.get_parameter("lane_id_rgba").value)
        self.arrow_rgba = tuple(float(v) for v in self.get_parameter("arrow_rgba").value)
        self.signal_group_rgba = tuple(float(v) for v in self.get_parameter("signal_group_rgba").value)

        self.debug_log_lanes = bool(self.get_parameter("debug_log_lanes").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(MapData, input_topic, self.map_callback, qos)
        self.pub = self.create_publisher(MarkerArray, output_topic, qos)

        self.get_logger().info(f"Subscribed to MAP topic: {input_topic}")
        self.get_logger().info(f"Publishing MarkerArray to: {output_topic}")

    def _make_header(self, stamp, frame_id: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def _frame_id_for_intersection(self, intersection_id: int) -> str:
        return f"{self.frame_namespace}{intersection_id}"

    def _selected_intersections(self, msg: MapData) -> List[MapIntersection]:
        if self.publish_all_intersections:
            return list(msg.intersections)

        return [
            intersection
            for intersection in msg.intersections
            if int(intersection.id.id) == self.target_intersection_id
        ]

    def _node_type_scale(self, node_type: str) -> float:
        scales = {
            "node-XY1": 0.01,
            "node-XY2": 0.1,
            "node-XY3": 1.0,
            "node-XY4": 10.0,
            "node-XY5": 0.1,
            "node-XY6": 0.01,
            "computed": 1.0,
        }
        return scales.get(node_type, 1.0)

    def _transform_xy(self, x: float, y: float) -> Tuple[float, float]:
        if self.swap_xy:
            x, y = y, x

        if self.invert_x:
            x = -x

        if self.invert_y:
            y = -y

        x = x * self.node_scale_x + self.node_offset_x
        y = y * self.node_scale_y + self.node_offset_y
        return x, y

    def _lane_points(
        self,
        intersection: MapIntersection,
        lane: MapLane,
        z_value: float,
    ) -> List[Point]:
        points: List[Point] = []

        # RViz tarafında lokal intersection frame kullanıyoruz.
        # ref_point absolute anchor; lane nodes ise delta.
        # Bu yüzden kümülatif delta ile lane polyline oluşturuyoruz.
        current_x = 0.0
        current_y = 0.0

        # İlk anchor noktası
        start_x, start_y = self._transform_xy(0.0, 0.0)
        start_pt = Point()
        start_pt.x = start_x
        start_pt.y = start_y
        start_pt.z = z_value
        points.append(start_pt)

        for node in lane.nodes:
            node_type = str(node.node_type)
            scale = self._node_type_scale(node_type)

            dx = float(node.x) * scale
            dy = float(node.y) * scale

            current_x += dx
            current_y += dy

            px, py = self._transform_xy(current_x, current_y)

            pt = Point()
            pt.x = px
            pt.y = py
            pt.z = z_value
            points.append(pt)

        # Tek nokta varsa marker sorun çıkarabilir; yine de en az 2 nokta verelim
        if len(points) == 1:
            duplicate = Point()
            duplicate.x = points[0].x
            duplicate.y = points[0].y
            duplicate.z = points[0].z
            points.append(duplicate)

        if self.debug_log_lanes:
            self.get_logger().info(
                f"lane_id={int(lane.lane_id)} points={[(round(p.x, 3), round(p.y, 3)) for p in points]}"
            )

        return points

    def _lane_midpoint(
        self,
        intersection: MapIntersection,
        lane: MapLane,
        z_value: float,
    ) -> Point:
        points = self._lane_points(intersection, lane, z_value)
        if not points:
            pt = Point()
            pt.z = z_value
            return pt
        return points[len(points) // 2]

    def _make_lane_line_marker(
        self,
        intersection: MapIntersection,
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

        marker.points = self._lane_points(intersection, lane, self.line_z)
        marker.pose.orientation.w = 1.0
        return marker

    def _make_lane_text_marker(
        self,
        intersection: MapIntersection,
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
        marker.pose.position = self._lane_midpoint(intersection, lane, self.lane_id_z)
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
        intersection: MapIntersection,
        lane: MapLane,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Optional[Marker]:
        points = self._lane_points(intersection, lane, self.arrow_z)
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
        intersection: MapIntersection,
        from_lane: MapLane,
        to_lane: MapLane,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Optional[Marker]:
        from_points = self._lane_points(intersection, from_lane, self.connection_z)
        to_points = self._lane_points(intersection, to_lane, self.connection_z)

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
        intersection: MapIntersection,
        from_lane: MapLane,
        to_lane: MapLane,
        signal_group: int,
        stamp,
        marker_id: int,
        frame_id: str,
        namespace_suffix: str,
    ) -> Optional[Marker]:
        from_points = self._lane_points(intersection, from_lane, self.signal_group_z)
        to_points = self._lane_points(intersection, to_lane, self.signal_group_z)

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
                        intersection=intersection,
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
                            intersection=intersection,
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
                        intersection=intersection,
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
                                intersection=intersection,
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
                                intersection=intersection,
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