#!/usr/bin/env python3

import socket
import traceback

import rclpy
from rclpy.node import Node

from pycmssdk.asn1 import Asn1Type, asn1_decode

from v2i_sdsm_msgs.msg import SdsmPacket
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class SdsmUdpBridge(Node):

    def __init__(self):
        super().__init__("sdsm_udp_bridge")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 7111)
        self.declare_parameter("recv_buf", 4096)
        self.declare_parameter("topic", "/sdsm")

        bind_ip = self.get_parameter("bind_ip").value
        udp_port = int(self.get_parameter("udp_port").value)

        topic = self.get_parameter("topic").value

        # QoS (broadcast-like)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(SdsmPacket, topic, qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, udp_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.001, self.poll_udp)

        self.get_logger().info(f"SDSM UDP bridge listening on {bind_ip}:{udp_port}")

    def poll_udp(self):

        try:
            data, addr = self.sock.recvfrom(4096)
        except BlockingIOError:
            return

        try:
            decoded = asn1_decode(data, Asn1Type.US_MESSAGE_FRAME)
        except Exception as e:
            self.get_logger().warn(f"ASN decode failed: {e}")
            return

        try:
            ros_msg = self.convert_decoded_to_ros(decoded)
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        self.pub.publish(ros_msg)

    def convert_decoded_to_ros(self, decoded):

        msg = SdsmPacket()

        msg.message_id = int(decoded.get("messageId", 0))

        value = decoded.get("value")

        msg_type = value[0]
        payload = value[1]

        if msg_type != "SensorDataSharingMessage":
            raise ValueError("Not SDSM")

        msg.msg_count = payload.get("msgCnt", 0)
        msg.source_id = payload.get("sourceID", b'').decode()

        # objects parsing would go here

        return msg


def main():

    rclpy.init()

    node = SdsmUdpBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()