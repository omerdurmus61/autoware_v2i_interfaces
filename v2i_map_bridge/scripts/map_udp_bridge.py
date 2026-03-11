#!/usr/bin/env python3

import socket
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from pycmssdk.asn1 import Asn1Type, asn1_decode

from v2i_map_msgs.msg import MapData


class MapUdpBridge(Node):

    def __init__(self):
        super().__init__("map_udp_bridge")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 7113)
        self.declare_parameter("recv_buf", 4096)
        self.declare_parameter("filter_hex_prefix", "0012")
        self.declare_parameter("topic", "/map")
        self.declare_parameter("log_every_n", 10)

        bind_ip = self.get_parameter("bind_ip").value
        udp_port = int(self.get_parameter("udp_port").value)
        self.recv_buf = int(self.get_parameter("recv_buf").value)
        self.filter_hex_prefix = str(self.get_parameter("filter_hex_prefix").value).lower()
        topic = str(self.get_parameter("topic").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

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
            f"Log every N={self.log_every_n}"
        )

        self.timer = self.create_timer(0.001, self.poll_udp)

    def poll_udp(self):
        try:
            data, addr = self.sock.recvfrom(self.recv_buf)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().error(f"UDP recv error: {e}")
            return

        if self.filter_hex_prefix:
            try:
                if data.hex()[:len(self.filter_hex_prefix)] != self.filter_hex_prefix:
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

    def convert_decoded_to_ros(self, decoded):
        msg = MapData()
        msg.message_id = int(decoded.get("messageId", 0)) & 0xFF

        value = decoded.get("value", None)
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise ValueError(f"Unexpected 'value' format: {type(value)} -> {value}")

        msg_type = value[0]
        payload = value[1]

        if msg_type != "MapData":
            raise ValueError(f"Not a MAP payload. value[0]={msg_type}")

        msg.msg_issue_revision = int(payload.get("msgIssueRevision", 0)) & 0xFF

        # Fill intersections here later

        return msg


def main():
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