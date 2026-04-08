#!/usr/bin/env python3
import socket
import traceback
from typing import Any, Dict, List, Tuple, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from pycmssdk.asn1 import Asn1Type, asn1_decode

from v2i_spat_msgs.msg import (
    SpatPacket,
    Spat,
    IntersectionState,
    MovementState,
    MovementEvent,
    TimeChangeDetails,
)


DecodedType = Dict[str, Any]


def map_event_state(event_state_str: Any) -> int:
    """
    Maps eventState strings (from your decoded output) to MovementEvent enum values.
    Extend this mapping as you observe more states.
    """
    if not isinstance(event_state_str, str):
        return MovementEvent.EVENT_UNKNOWN

    s = event_state_str.strip()

    if s == "stop-And-Remain":
        return MovementEvent.STOP_AND_REMAIN
    if s == "protected-Movement-Allowed":
        return MovementEvent.PROTECTED_MOVEMENT_ALLOWED
    if s == "permissive-Movement-Allowed":
        return MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED
    if s == "protected-clearance":
        return MovementEvent.PROTECTED_CLEARANCE

    return MovementEvent.EVENT_UNKNOWN


class SpatUdpBridge(Node):
    def __init__(self):
        super().__init__("spat_udp_bridge")

        # Parameters
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 7114)
        self.declare_parameter("recv_buf", 2048)
        self.declare_parameter("filter_hex_prefix", "0013")  # SPaT filter (first 2 bytes)
        self.declare_parameter("topic", "/v2i/spat/raw")
        self.declare_parameter("log_every_n", 1)  # log every N decoded messages (0 disables)

        bind_ip = self.get_parameter("bind_ip").value
        udp_port = int(self.get_parameter("udp_port").value)
        self.recv_buf = int(self.get_parameter("recv_buf").value)
        self.filter_hex_prefix = str(self.get_parameter("filter_hex_prefix").value).lower()
        topic = str(self.get_parameter("topic").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        # QoS (broadcast-like)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(SpatPacket, topic, qos)

        # UDP socket
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

        # Poll socket periodically (non-blocking)
        self.timer = self.create_timer(0.001, self.poll_udp)

    def poll_udp(self):
        try:
            data, addr = self.sock.recvfrom(self.recv_buf)
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().error(f"UDP recv error: {e}")
            return

        # Optional SPaT filter by hex prefix (same idea as your test2.py)
        if self.filter_hex_prefix:
            try:
                if data.hex()[: len(self.filter_hex_prefix)] != self.filter_hex_prefix:
                    return
            except Exception:
                return

        # ASN.1 decode (binary -> python dict/tuple)
        try:
            decoded = asn1_decode(data, Asn1Type.US_MESSAGE_FRAME)
        except Exception as e:
            self.get_logger().warn(f"ASN.1 decode failed: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        self.msg_counter += 1

        # Optional logging of the decoded structure
        if self.log_every_n > 0 and (self.msg_counter % self.log_every_n) == 0:
            self.get_logger().info(
                f"Decoded message #{self.msg_counter} from {addr[0]}:{addr[1]}:\n{decoded}"
            )

        # Convert decoded -> ROS msg
        try:
            ros_msg = self.convert_decoded_to_ros(decoded)
        except Exception as e:
            self.get_logger().error(f"Convert-to-ROS failed: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        # Publish
        self.pub.publish(ros_msg)

    def convert_decoded_to_ros(self, decoded: Any) -> SpatPacket:

        if not isinstance(decoded, dict):
            raise ValueError(f"Decoded object is not a dict: {type(decoded)}")

        out = SpatPacket()
        out.message_id = int(decoded.get("messageId", 0)) & 0xFF

        value = decoded.get("value", None)
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise ValueError(f"Unexpected 'value' format: {type(value)} -> {value}")

        msg_type = value[0]
        payload = value[1]

        if msg_type != "SPAT":
            raise ValueError(f"Not a SPAT payload. value[0]={msg_type}")

        if not isinstance(payload, dict):
            raise ValueError(f"SPAT payload is not a dict: {type(payload)}")

        spat = Spat()
        spat.time_stamp = int(payload.get("timeStamp", 0)) & 0xFFFFFFFF

        intersections = payload.get("intersections", [])
        if intersections is None:
            intersections = []
        if not isinstance(intersections, list):
            raise ValueError("payload['intersections'] is not a list")

        for inter in intersections:
            if not isinstance(inter, dict):
                continue

            inter_msg = IntersectionState()

            # intersection_id = inter['id']['id']
            inter_id_obj = inter.get("id", {})
            inter_id = 0
            if isinstance(inter_id_obj, dict):
                inter_id = inter_id_obj.get("id", 0)
            inter_msg.intersection_id = int(inter_id) & 0xFFFF

            inter_msg.revision = int(inter.get("revision", 0)) & 0xFF
            inter_msg.moy = int(inter.get("moy", 0)) & 0xFFFFFFFF
            inter_msg.time_stamp = int(inter.get("timeStamp", 0)) & 0xFFFF

            # status tuple -> (value, bit_length)
            status = inter.get("status", None)
            if isinstance(status, (tuple, list)) and len(status) >= 2:
                inter_msg.status_value = int(status[0]) & 0xFFFFFFFF
                inter_msg.status_bit_length = int(status[1]) & 0xFFFF
            else:
                inter_msg.status_value = 0
                inter_msg.status_bit_length = 0

            # states[]
            states = inter.get("states", [])
            if states is None:
                states = []
            if not isinstance(states, list):
                states = []

            for st in states:
                if not isinstance(st, dict):
                    continue

                st_msg = MovementState()
                st_msg.signal_group = int(st.get("signalGroup", 0)) & 0xFF

                events = st.get("state-time-speed", [])
                if events is None:
                    events = []
                if not isinstance(events, list):
                    events = []

                for ev in events:
                    if not isinstance(ev, dict):
                        continue

                    ev_msg = MovementEvent()
                    ev_msg.event_state = map_event_state(ev.get("eventState", ""))

                    timing = ev.get("timing", {})
                    t_msg = TimeChangeDetails()

                    if isinstance(timing, dict) and "minEndTime" in timing:
                        t_msg.has_min_end_time = True
                        t_msg.min_end_time = int(timing.get("minEndTime", 0)) & 0xFFFF
                    else:
                        t_msg.has_min_end_time = False
                        t_msg.min_end_time = 0

                    if isinstance(timing, dict) and "maxEndTime" in timing:
                        t_msg.has_max_end_time = True
                        t_msg.max_end_time = int(timing.get("maxEndTime", 0)) & 0xFFFF
                    else:
                        t_msg.has_max_end_time = False
                        t_msg.max_end_time = 0

                    ev_msg.timing = t_msg
                    st_msg.events.append(ev_msg)

                inter_msg.states.append(st_msg)

            spat.intersections.append(inter_msg)

        out.spat = spat
        return out


def main():
    rclpy.init()
    node = SpatUdpBridge()
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