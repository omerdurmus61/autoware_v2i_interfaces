#!/usr/bin/env python3

from collections import deque
from copy import deepcopy
from typing import Deque, Optional

import rclpy
from autoware_perception_msgs.msg import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String


class V2XSpatFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("v2x_spat_fusion_node")

        self.declare_parameter(
            "camera_input_topic",
            "/traffic_light_hsv_roi_classifier/output/traffic_signals",
        )
        self.declare_parameter("spat_input_topic", "/v2i/spat/traffic_signals")
        self.declare_parameter(
            "output_topic",
            "/perception/traffic_light_recognition/traffic_signals",
        )
        self.declare_parameter(
            "debug_source_topic",
            "/v2i_spat_camera_fusion/debug/fusion_source",
        )
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("camera_max_latency", 0.5)
        self.declare_parameter("v2x_max_latency", 1.0)
        self.declare_parameter("camera_reliability_threshold", 0.5)
        self.declare_parameter("v2x_reliability_threshold", 0.5)
        self.declare_parameter("camera_expected_rate", 10.0)
        self.declare_parameter("v2x_expected_rate", 10.0)
        self.declare_parameter("default_safe_state", "RED")

        camera_input_topic = str(self.get_parameter("camera_input_topic").value)
        spat_input_topic = str(self.get_parameter("spat_input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        debug_source_topic = str(self.get_parameter("debug_source_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.camera_max_latency = float(
            self.get_parameter("camera_max_latency").value
        )
        self.v2x_max_latency = float(self.get_parameter("v2x_max_latency").value)
        self.camera_reliability_threshold = float(
            self.get_parameter("camera_reliability_threshold").value
        )
        self.v2x_reliability_threshold = float(
            self.get_parameter("v2x_reliability_threshold").value
        )
        self.camera_expected_rate = float(
            self.get_parameter("camera_expected_rate").value
        )
        self.v2x_expected_rate = float(self.get_parameter("v2x_expected_rate").value)
        self.default_safe_state = self._normalize_state_name(
            str(self.get_parameter("default_safe_state").value)
        )

        if publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "publish_rate_hz <= 0.0 is invalid. Falling back to 10.0 Hz."
            )
            publish_rate_hz = 10.0

        qos_input = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_output = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.camera_sub = self.create_subscription(
            TrafficLightGroupArray,
            camera_input_topic,
            self.camera_callback,
            qos_input,
        )
        self.spat_sub = self.create_subscription(
            TrafficLightGroupArray,
            spat_input_topic,
            self.spat_callback,
            qos_input,
        )
        self.output_pub = self.create_publisher(
            TrafficLightGroupArray,
            output_topic,
            qos_output,
        )
        self.debug_source_pub = self.create_publisher(
            String,
            debug_source_topic,
            qos_output,
        )
        self.publish_timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_fused_output,
        )

        self.latest_camera_msg: Optional[TrafficLightGroupArray] = None
        self.latest_spat_msg: Optional[TrafficLightGroupArray] = None
        self.camera_arrival_history: Deque[float] = deque(maxlen=10)
        self.v2x_arrival_history: Deque[float] = deque(maxlen=10)

        self.get_logger().info(
            "Reliability-aware rule-based camera/SPaT fusion enabled."
        )
        self.get_logger().info(f"camera_input_topic: {camera_input_topic}")
        self.get_logger().info(f"spat_input_topic: {spat_input_topic}")
        self.get_logger().info(f"output_topic: {output_topic}")
        self.get_logger().info(f"debug_source_topic: {debug_source_topic}")
        self.get_logger().info(f"publish_rate_hz: {publish_rate_hz:.2f}")

    def camera_callback(self, msg: TrafficLightGroupArray) -> None:
        self.latest_camera_msg = deepcopy(msg)
        self._record_arrival(self.camera_arrival_history)

    def spat_callback(self, msg: TrafficLightGroupArray) -> None:
        self.latest_spat_msg = deepcopy(msg)
        self._record_arrival(self.v2x_arrival_history)

    def publish_fused_output(self) -> None:
        output_msg = TrafficLightGroupArray()
        output_msg.stamp = self.get_clock().now().to_msg()
        fused_groups = self._fuse_groups()
        output_msg.traffic_light_groups = fused_groups
        self.output_pub.publish(output_msg)
        debug_msg = String()
        debug_msg.data = self._debug_source_label()
        self.debug_source_pub.publish(debug_msg)

    def _fuse_groups(self) -> list[TrafficLightGroup]:
        camera_groups = self._group_map(self.latest_camera_msg)
        v2x_groups = self._group_map(self.latest_spat_msg)
        group_ids = sorted(set(camera_groups.keys()) | set(v2x_groups.keys()))

        fused_groups: list[TrafficLightGroup] = []
        for group_id in group_ids:
            camera_group = camera_groups.get(group_id)
            v2x_group = v2x_groups.get(group_id)
            fused_group, _ = self._fuse_single_group(group_id, camera_group, v2x_group)
            fused_groups.append(fused_group)
        return fused_groups

    def _fuse_single_group(
        self,
        group_id: int,
        camera_group: Optional[TrafficLightGroup],
        v2x_group: Optional[TrafficLightGroup],
    ) -> tuple[TrafficLightGroup, str]:
        camera_state = self._group_state(camera_group)
        v2x_state = self._group_state(v2x_group)

        camera_freshness = self._message_freshness(
            self.latest_camera_msg,
            self.camera_max_latency,
        )
        v2x_freshness = self._message_freshness(
            self.latest_spat_msg,
            self.v2x_max_latency,
        )
        camera_update_rate_score = self._update_rate_score(
            self.camera_arrival_history,
            self.camera_expected_rate,
        )
        v2x_update_rate_score = self._update_rate_score(
            self.v2x_arrival_history,
            self.v2x_expected_rate,
        )
        camera_confidence = self._camera_confidence(camera_group, camera_state)
        camera_reliability = self._camera_reliability(
            camera_confidence,
            camera_freshness,
            camera_update_rate_score,
        )
        id_match_score = self._id_match_score(group_id, camera_group, v2x_group)
        v2x_reliability = self._v2x_reliability(
            v2x_freshness,
            id_match_score,
            v2x_update_rate_score,
        )

        camera_reliable = camera_reliability >= self.camera_reliability_threshold
        v2x_reliable = v2x_reliability >= self.v2x_reliability_threshold
        v2x_fresh = v2x_freshness > 0.0

        # Decision order:
        # 1) If V2I is fresh and reliable for this group, always trust V2I.
        # 2) Otherwise, if the camera is reliable, fall back to the camera.
        # 3) If neither source is usable, publish UNKNOWN.
        if v2x_group is not None and v2x_reliable and v2x_fresh:
            selected_state = v2x_state
            selected_source = "v2i"
        elif camera_group is not None and camera_reliable:
            selected_state = camera_state
            selected_source = "camera"
        else:
            selected_state = "UNKNOWN"
            selected_source = "unknown"

        confidence = max(camera_reliability, v2x_reliability)
        if selected_state == "UNKNOWN":
            confidence = 0.0

        return self._build_group(group_id, selected_state, confidence), selected_source

    def _group_map(
        self, msg: Optional[TrafficLightGroupArray]
    ) -> dict[int, TrafficLightGroup]:
        if msg is None:
            return {}
        return {
            int(group.traffic_light_group_id): group
            for group in msg.traffic_light_groups
        }

    def _record_arrival(self, history: Deque[float]) -> None:
        history.append(self.get_clock().now().nanoseconds / 1e9)

    def _debug_source_label(self) -> str:
        if self._has_usable_v2x():
            return "v2i"
        return "camera"

    def _has_usable_v2x(self) -> bool:
        v2x_groups = self._group_map(self.latest_spat_msg)
        if not v2x_groups:
            return False

        v2x_freshness = self._message_freshness(
            self.latest_spat_msg,
            self.v2x_max_latency,
        )
        v2x_update_rate_score = self._update_rate_score(
            self.v2x_arrival_history,
            self.v2x_expected_rate,
        )

        for group_id, v2x_group in v2x_groups.items():
            camera_group = self._group_map(self.latest_camera_msg).get(group_id)
            id_match_score = self._id_match_score(group_id, camera_group, v2x_group)
            v2x_reliability = self._v2x_reliability(
                v2x_freshness,
                id_match_score,
                v2x_update_rate_score,
            )
            if v2x_reliability >= self.v2x_reliability_threshold:
                return True
        return False

    def _latency_seconds(self, msg: Optional[TrafficLightGroupArray]) -> Optional[float]:
        if msg is None:
            return None
        stamp_ns = self._stamp_nanoseconds(msg.stamp)
        if stamp_ns is None:
            return None
        latency = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return max(0.0, latency)

    def _message_freshness(
        self,
        msg: Optional[TrafficLightGroupArray],
        max_latency: float,
    ) -> float:
        latency = self._latency_seconds(msg)
        if latency is None:
            return 0.0
        return self._freshness_score(latency, max_latency)

    def _freshness_score(self, latency: float, max_latency: float) -> float:
        if max_latency <= 0.0:
            return 0.0
        return max(0.0, 1.0 - (latency / max_latency))

    def _update_rate_hz(self, history: Deque[float]) -> float:
        if len(history) < 2:
            return 0.0
        deltas = [
            curr - prev
            for prev, curr in zip(list(history)[:-1], list(history)[1:])
            if curr > prev
        ]
        if not deltas:
            return 0.0
        mean_delta = sum(deltas) / len(deltas)
        if mean_delta <= 0.0:
            return 0.0
        return 1.0 / mean_delta

    def _update_rate_score(self, history: Deque[float], expected_rate: float) -> float:
        if len(history) < 2 or expected_rate <= 0.0:
            return 1.0
        return min(1.0, self._update_rate_hz(history) / expected_rate)

    def _camera_confidence(
        self,
        group: Optional[TrafficLightGroup],
        state: str,
    ) -> float:
        if group is None:
            return 0.0
        confidences = [
            float(getattr(element, "confidence", 0.0)) for element in group.elements
        ]
        confidence = max(confidences, default=0.0)
        if confidence > 0.0:
            return max(0.0, min(1.0, confidence))
        if state == "UNKNOWN":
            return 0.0
        if state in {"RED", "GREEN", "AMBER"}:
            return 1.0
        return 0.0

    def _camera_reliability(
        self,
        camera_confidence: float,
        camera_freshness: float,
        update_rate_score: float,
    ) -> float:
        return max(
            0.0,
            min(1.0, camera_confidence * camera_freshness * update_rate_score),
        )

    def _v2x_reliability(
        self,
        v2x_freshness: float,
        id_match_score: float,
        update_rate_score: float,
    ) -> float:
        return max(0.0, min(1.0, v2x_freshness * id_match_score * update_rate_score))

    def _id_match_score(
        self,
        group_id: int,
        camera_group: Optional[TrafficLightGroup],
        v2x_group: Optional[TrafficLightGroup],
    ) -> float:
        if v2x_group is None:
            return 0.0
        v2x_group_id = int(v2x_group.traffic_light_group_id)
        if camera_group is None:
            return 1.0 if v2x_group_id > 0 else 0.0
        camera_group_id = int(camera_group.traffic_light_group_id)
        return 1.0 if camera_group_id == v2x_group_id == group_id else 0.0

    def _group_state(self, group: Optional[TrafficLightGroup]) -> str:
        if group is None or not group.elements:
            return "UNKNOWN"

        colors = [int(element.color) for element in group.elements]
        if TrafficLightElement.RED in colors:
            return "RED"
        if TrafficLightElement.AMBER in colors:
            return "AMBER"
        if TrafficLightElement.GREEN in colors:
            return "GREEN"
        return "UNKNOWN"

    def _build_group(
        self, group_id: int, state: str, confidence: float
    ) -> TrafficLightGroup:
        signal = TrafficLightGroup()
        signal.traffic_light_group_id = int(group_id)

        element = TrafficLightElement()
        element.shape = TrafficLightElement.CIRCLE
        element.status = TrafficLightElement.SOLID_ON
        element.confidence = float(max(0.0, min(1.0, confidence)))

        if state == "RED":
            element.color = TrafficLightElement.RED
        elif state == "GREEN":
            element.color = TrafficLightElement.GREEN
        elif state == "AMBER":
            element.color = TrafficLightElement.AMBER
        else:
            element.color = TrafficLightElement.UNKNOWN
            element.shape = TrafficLightElement.UNKNOWN
            element.status = TrafficLightElement.UNKNOWN
            element.confidence = 0.0

        signal.elements.append(element)
        return signal

    def _normalize_state_name(self, state: str) -> str:
        normalized = state.strip().upper()
        if normalized == "YELLOW":
            return "AMBER"
        if normalized in {"RED", "GREEN", "AMBER", "UNKNOWN"}:
            return normalized
        return "RED"

    def _stamp_nanoseconds(self, stamp) -> Optional[int]:
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        if sec == 0 and nanosec == 0:
            return None

        try:
            return Time.from_msg(stamp).nanoseconds
        except Exception:
            return (sec * 1_000_000_000) + nanosec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V2XSpatFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
