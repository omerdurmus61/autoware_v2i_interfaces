#!/usr/bin/env python3

import socket
import traceback
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from pycmssdk.asn1 import Asn1Type, asn1_decode

from v2i_map_msgs.msg import (
    MapConnection,
    MapData,
    MapIntersection,
    MapLane,
    MapLaneAttributes,
    MapNode,
)


class MapUdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("map_udp_bridge")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 7113)
        self.declare_parameter("recv_buf", 4096)
        self.declare_parameter("filter_hex_prefix", "0012")
        self.declare_parameter("topic", "/v2i/map/raw")
        self.declare_parameter("log_every_n", 10)
        self.declare_parameter("frame_id", "map")

        bind_ip = str(self.get_parameter("bind_ip").value)
        udp_port = int(self.get_parameter("udp_port").value)
        self.recv_buf = int(self.get_parameter("recv_buf").value)
        self.filter_hex_prefix = str(self.get_parameter("filter_hex_prefix").value).lower()
        topic = str(self.get_parameter("topic").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(MapData, topic, qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, udp_port))
        self.sock.setblocking(False)

        self.msg_counter = 0

        self.get_logger().info(f"Listening UDP on {bind_ip}:{udp_port} -> publishing {topic}")
        self.get_logger().info(
            f"Filter hex prefix: {self.filter_hex_prefix or '(none)'} | "
            f"Buffer: {self.recv_buf} bytes | "
            f"Log every N={self.log_every_n} | "
            f"Frame ID={self.frame_id}"
        )

        self.timer = self.create_timer(0.001, self.poll_udp)

    @staticmethod
    def _safe_int(value: Any, default: int = 0) -> int:
        try:
            if value is None:
                return default
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _tuple_value_and_bits(value: Any) -> Tuple[int, int]:
        if isinstance(value, (tuple, list)) and len(value) == 2:
            return MapUdpBridge._safe_int(value[0], 0), MapUdpBridge._safe_int(value[1], 0)
        return 0, 0

    @staticmethod
    def _extract_lane_type(value: Any) -> Tuple[str, int, int]:
        if isinstance(value, (tuple, list)) and len(value) == 2:
            lane_type_name = str(value[0])
            lane_type_value, lane_type_bits = MapUdpBridge._tuple_value_and_bits(value[1])
            return lane_type_name, lane_type_value, lane_type_bits
        return "", 0, 0

    @staticmethod
    def _extract_node_delta(node_entry: Dict[str, Any]) -> Tuple[str, int, int]:
        delta = node_entry.get("delta")

        if not (isinstance(delta, (tuple, list)) and len(delta) == 2):
            return "", 0, 0

        node_type = str(delta[0])
        node_data = delta[1] if isinstance(delta[1], dict) else {}

        x = MapUdpBridge._safe_int(node_data.get("x"), 0)
        y = MapUdpBridge._safe_int(node_data.get("y"), 0)
        return node_type, x, y

    @staticmethod
    def _extract_connecting_lane(connection_payload: Dict[str, Any]) -> int:
        connecting_lane = connection_payload.get("connectingLane", {})
        if isinstance(connecting_lane, dict):
            return MapUdpBridge._safe_int(connecting_lane.get("lane"), 0)
        return 0

    def poll_udp(self) -> None:
        try:
            data, addr = self.sock.recvfrom(self.recv_buf)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().error(f"UDP recv error: {e}")
            return

        if self.filter_hex_prefix:
            try:
                if data.hex()[: len(self.filter_hex_prefix)] != self.filter_hex_prefix:
                    return
            except Exception:
                return

        try:
            decoded = asn1_decode(data, Asn1Type.US_MESSAGE_FRAME)
        except Exception as e:
            self.get_logger().warn(f"ASN.1 decode failed: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        self.msg_counter += 1

        if self.log_every_n > 0 and (self.msg_counter % self.log_every_n) == 0:
            self.get_logger().info(
                f"Decoded message #{self.msg_counter} from {addr[0]}:{addr[1]}:\n{decoded}"
            )

        try:
            ros_msg = self.convert_decoded_to_ros(decoded)
        except Exception as e:
            self.get_logger().error(f"Convert-to-ROS failed: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        self.pub.publish(ros_msg)

    def _build_lane_attributes(self, src: Dict[str, Any]) -> MapLaneAttributes:
        attrs = MapLaneAttributes()

        directional_use_value, directional_use_bits = self._tuple_value_and_bits(
            src.get("directionalUse")
        )
        shared_with_value, shared_with_bits = self._tuple_value_and_bits(src.get("sharedWith"))
        lane_type_name, lane_type_value, lane_type_bits = self._extract_lane_type(src.get("laneType"))

        attrs.directional_use_value = directional_use_value & 0xFF
        attrs.directional_use_bits = directional_use_bits & 0xFF
        attrs.shared_with_value = shared_with_value & 0xFF
        attrs.shared_with_bits = shared_with_bits & 0xFF
        attrs.lane_type = lane_type_name
        attrs.lane_type_value = lane_type_value & 0xFF
        attrs.lane_type_bits = lane_type_bits & 0xFF

        return attrs

    def _build_nodes(self, node_list_value: Any) -> List[MapNode]:
        result: List[MapNode] = []

        if not (isinstance(node_list_value, (tuple, list)) and len(node_list_value) == 2):
            return result

        node_list_kind = node_list_value[0]
        node_entries = node_list_value[1]

        if node_list_kind != "nodes" or not isinstance(node_entries, list):
            return result

        for node_entry in node_entries:
            if not isinstance(node_entry, dict):
                continue

            node_type, x, y = self._extract_node_delta(node_entry)

            node_msg = MapNode()
            node_msg.node_type = node_type
            node_msg.x = x
            node_msg.y = y
            result.append(node_msg)

        return result

    def _build_connections(self, connects_to_value: Any) -> List[MapConnection]:
        result: List[MapConnection] = []

        if not isinstance(connects_to_value, list):
            return result

        for conn_entry in connects_to_value:
            if not isinstance(conn_entry, dict):
                continue

            conn_msg = MapConnection()
            conn_msg.connecting_lane = self._extract_connecting_lane(conn_entry) & 0xFFFF
            conn_msg.signal_group = self._safe_int(conn_entry.get("signalGroup"), 0) & 0xFFFF
            result.append(conn_msg)

        return result

    def _build_lane(self, src: Dict[str, Any]) -> MapLane:
        lane = MapLane()

        lane.lane_id = self._safe_int(src.get("laneID"), 0) & 0xFFFF
        lane.lane_attributes = self._build_lane_attributes(src.get("laneAttributes", {}))

        maneuvers_value, maneuvers_bits = self._tuple_value_and_bits(src.get("maneuvers"))
        lane.maneuvers_value = maneuvers_value & 0xFFFF
        lane.maneuvers_bits = maneuvers_bits & 0xFF

        lane.nodes = self._build_nodes(src.get("nodeList"))
        lane.connections = self._build_connections(src.get("connectsTo", []))

        return lane

    def _build_intersection(self, src: Dict[str, Any]) -> MapIntersection:
        intersection = MapIntersection()

        intersection.name = str(src.get("name", ""))

        intersection_id = src.get("id", {})
        if isinstance(intersection_id, dict):
            intersection.id.id = self._safe_int(intersection_id.get("id"), 0) & 0xFFFFFFFF

        intersection.revision = self._safe_int(src.get("revision"), 0) & 0xFFFF

        ref_point = src.get("refPoint", {})
        if isinstance(ref_point, dict):
            intersection.ref_point.lat = self._safe_int(ref_point.get("lat"), 0)
            intersection.ref_point.lon = self._safe_int(
                ref_point.get("lon", ref_point.get("long", 0)),
                0,
            )

        lane_set = src.get("laneSet", [])
        if isinstance(lane_set, list):
            intersection.lane_set = [self._build_lane(lane_src) for lane_src in lane_set if isinstance(lane_src, dict)]

        return intersection

    def convert_decoded_to_ros(self, decoded: Dict[str, Any]) -> MapData:
        msg = MapData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.message_id = self._safe_int(decoded.get("messageId"), 0) & 0xFF

        value = decoded.get("value", None)
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise ValueError(f"Unexpected 'value' format: {type(value)} -> {value}")

        msg_type = value[0]
        payload = value[1]

        if msg_type != "MapData":
            raise ValueError(f"Not a MAP payload. value[0]={msg_type}")

        if not isinstance(payload, dict):
            raise ValueError(f"Unexpected MAP payload type: {type(payload)}")

        msg.msg_issue_revision = self._safe_int(payload.get("msgIssueRevision"), 0) & 0xFF

        intersections = payload.get("intersections", [])
        if isinstance(intersections, list):
            msg.intersections = [
                self._build_intersection(intersection_src)
                for intersection_src in intersections
                if isinstance(intersection_src, dict)
            ]

        return msg


def main() -> None:
    rclpy.init()
    node = MapUdpBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.sock.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()