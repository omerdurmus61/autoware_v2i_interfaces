#!/usr/bin/env python3

import socket
import traceback
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from pycmssdk.asn1 import Asn1Type, asn1_decode

from v2i_sdsm_msgs.msg import SDSM, SDSMTimestamp, SDSMDetectedObject


class SdsmUdpBridge(Node):
    def __init__(self):
        super().__init__("sdsm_udp_bridge")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 7112)
        self.declare_parameter("recv_buf", 4096)
        self.declare_parameter("filter_hex_prefix", "0029")
        self.declare_parameter("topic", "/v2i/sdsm/raw")
        self.declare_parameter("log_every_n", 10)
        self.declare_parameter("frame_id", "")

        bind_ip = str(self.get_parameter("bind_ip").value)
        udp_port = int(self.get_parameter("udp_port").value)
        self.recv_buf = int(self.get_parameter("recv_buf").value)
        self.filter_hex_prefix = str(self.get_parameter("filter_hex_prefix").value).lower()
        topic = str(self.get_parameter("topic").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(SDSM, topic, qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, udp_port))
        self.sock.setblocking(False)

        self.msg_counter = 0

        self.timer = self.create_timer(0.001, self.poll_udp)

        self.get_logger().info(
            f"SDSM UDP bridge listening on {bind_ip}:{udp_port} -> publishing {topic}"
        )
        self.get_logger().info(
            f"Filter hex prefix: {self.filter_hex_prefix or '(none)'} | "
            f"Buffer: {self.recv_buf} bytes | "
            f"Log every N={self.log_every_n}"
        )

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
                if data.hex()[: len(self.filter_hex_prefix)] != self.filter_hex_prefix:
                    return
            except Exception:
                return

        try:
            decoded = asn1_decode(data, Asn1Type.US_MESSAGE_FRAME)
        except Exception as e:
            self.get_logger().warn(f"ASN decode failed: {e}")
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
            self.get_logger().error(f"Conversion failed: {e}")
            self.get_logger().debug(traceback.format_exc())
            return

        self.pub.publish(ros_msg)

    def _safe_int(self, value: Any, default: int = 0) -> int:
        try:
            return int(value)
        except Exception:
            return default

    def _safe_str(self, value: Any, default: str = "") -> str:
        if value is None:
            return default
        if isinstance(value, bytes):
            try:
                return value.decode(errors="ignore")
            except Exception:
                return default
        return str(value)

    def convert_decoded_to_ros(self, decoded: Any) -> SDSM:
        if not isinstance(decoded, dict):
            raise ValueError(f"Decoded object is not a dict: {type(decoded)}")

        msg = SDSM()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.message_id = self._safe_int(decoded.get("messageId", 0)) & 0xFF

        value = decoded.get("value", None)
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise ValueError(f"Unexpected 'value' format: {type(value)} -> {value}")

        msg_type = value[0]
        payload = value[1]

        if msg_type != "SensorDataSharingMessage":
            raise ValueError(f"Not SDSM. value[0]={msg_type}")

        if not isinstance(payload, dict):
            raise ValueError(f"SDSM payload is not a dict: {type(payload)}")

        msg.msg_count = self._safe_int(payload.get("msgCnt", 0)) & 0xFF
        msg.source_id = self._safe_str(payload.get("sourceID", b""))
        msg.equipment_type = self._safe_str(payload.get("equipmentType", ""))

        # Timestamp
        timestamp = payload.get("sDSMTimeStamp", {})
        if isinstance(timestamp, dict):
            ts_msg = SDSMTimestamp()
            ts_msg.year = self._safe_int(timestamp.get("year", 0)) & 0xFFFF
            ts_msg.month = self._safe_int(timestamp.get("month", 0)) & 0xFF
            ts_msg.day = self._safe_int(timestamp.get("day", 0)) & 0xFF
            ts_msg.hour = self._safe_int(timestamp.get("hour", 0)) & 0xFF
            ts_msg.minute = self._safe_int(timestamp.get("minute", 0)) & 0xFF
            ts_msg.second = self._safe_int(timestamp.get("second", 0)) & 0xFFFF
            ts_msg.offset = self._safe_int(timestamp.get("offset", 0))
            msg.sdsm_timestamp = ts_msg

        # Reference position
        ref_pos = payload.get("refPos", {})
        if isinstance(ref_pos, dict):
            msg.ref_lat = self._safe_int(ref_pos.get("lat", 0))
            msg.ref_lon = self._safe_int(ref_pos.get("long", 0))

        # Reference position confidence
        ref_pos_xy_conf = payload.get("refPosXYConf", {})
        if isinstance(ref_pos_xy_conf, dict):
            msg.ref_pos_semi_major = self._safe_int(ref_pos_xy_conf.get("semiMajor", 0)) & 0xFFFF
            msg.ref_pos_semi_minor = self._safe_int(ref_pos_xy_conf.get("semiMinor", 0)) & 0xFFFF
            msg.ref_pos_orientation = self._safe_int(ref_pos_xy_conf.get("orientation", 0)) & 0xFFFF

        # Objects
        objects = payload.get("objects", [])
        if objects is None:
            objects = []

        if not isinstance(objects, list):
            raise ValueError("payload['objects'] is not a list")

        for obj in objects:
            if not isinstance(obj, dict):
                continue

            det_obj_common = obj.get("detObjCommon", {})
            if not isinstance(det_obj_common, dict):
                continue

            ros_obj = SDSMDetectedObject()

            obj_type = self._safe_str(det_obj_common.get("objType", ""))
            if obj_type == "vehicle":
                ros_obj.object_type = SDSMDetectedObject.OBJECT_TYPE_VEHICLE
            elif obj_type == "vru":
                ros_obj.object_type = SDSMDetectedObject.OBJECT_TYPE_VRU
            else:
                ros_obj.object_type = SDSMDetectedObject.OBJECT_TYPE_UNKNOWN

            ros_obj.object_type_confidence = self._safe_int(
                det_obj_common.get("objTypeCfd", 0)
            ) & 0xFF

            ros_obj.object_id = self._safe_int(det_obj_common.get("objectID", 0)) & 0xFFFF
            ros_obj.measurement_time = self._safe_int(
                det_obj_common.get("measurementTime", 0)
            ) & 0xFFFF
            ros_obj.time_confidence = self._safe_str(
                det_obj_common.get("timeConfidence", "")
            )

            pos = det_obj_common.get("pos", {})
            if isinstance(pos, dict):
                ros_obj.position.offset_x = self._safe_int(pos.get("offsetX", 0))
                ros_obj.position.offset_y = self._safe_int(pos.get("offsetY", 0))

            pos_conf = det_obj_common.get("posConfidence", {})
            if isinstance(pos_conf, dict):
                pos_value = self._safe_str(pos_conf.get("pos", ""))
                elev_value = self._safe_str(pos_conf.get("elevation", ""))
                ros_obj.position_confidence = f"pos:{pos_value},elevation:{elev_value}"

            ros_obj.speed = self._safe_int(det_obj_common.get("speed", 0)) & 0xFFFF
            ros_obj.speed_confidence = self._safe_str(
                det_obj_common.get("speedConfidence", "")
            )

            ros_obj.heading = self._safe_int(det_obj_common.get("heading", 0)) & 0xFFFF
            ros_obj.heading_confidence = self._safe_str(
                det_obj_common.get("headingConf", "")
            )

            # Optional object data
            det_obj_opt_data = obj.get("detObjOptData", None)
            if isinstance(det_obj_opt_data, (tuple, list)) and len(det_obj_opt_data) == 2:
                opt_type = det_obj_opt_data[0]
                opt_payload = det_obj_opt_data[1]

                if opt_type == "detVeh" and isinstance(opt_payload, dict):
                    size = opt_payload.get("size", {})
                    if isinstance(size, dict):
                        ros_obj.width = self._safe_int(size.get("width", 0)) & 0xFFFF
                        ros_obj.length = self._safe_int(size.get("length", 0)) & 0xFFFF

                elif opt_type == "detVRU" and isinstance(opt_payload, dict):
                    ros_obj.vru_basic_type = self._safe_str(opt_payload.get("basicType", ""))

                    propulsion = opt_payload.get("propulsion", "")
                    if isinstance(propulsion, (tuple, list)) and len(propulsion) == 2:
                        ros_obj.vru_propulsion = (
                            f"{self._safe_str(propulsion[0])}:{self._safe_str(propulsion[1])}"
                        )
                    else:
                        ros_obj.vru_propulsion = self._safe_str(propulsion)

                    ros_obj.vru_radius = self._safe_int(opt_payload.get("radius", 0)) & 0xFFFF

            msg.objects.append(ros_obj)

        return msg


def main():
    rclpy.init()
    node = SdsmUdpBridge()

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