#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import DurabilityPolicy

from v2i_spat_msgs.msg import SpatPacket, MovementEvent
from autoware_perception_msgs.msg import (
    TrafficLightGroupArray,
    TrafficLightGroup,
    TrafficLightElement,
)


@dataclass
class CachedSignalState:
    intersection_id: int
    signal_group: int
    event_state: int
    last_update_ns: int
    min_end_time: int = 0
    max_end_time: int = 0
    has_min_end_time: bool = False
    has_max_end_time: bool = False


class V2ITrafficLightStatusPublisher(Node):
    def __init__(self) -> None:
        super().__init__("v2i_traffic_light_status_publisher")

        self.declare_parameter("input_topic", "/v2i/spat/raw")
        self.declare_parameter(
            "output_topic",
            "/perception/traffic_light_recognition/traffic_signals",
        )
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("signal_timeout_sec", 0.75)
        self.declare_parameter("keep_last_valid_on_unknown", True)
        self.declare_parameter("log_cache_details", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.signal_timeout_sec = float(self.get_parameter("signal_timeout_sec").value)
        self.keep_last_valid_on_unknown = bool(
            self.get_parameter("keep_last_valid_on_unknown").value
        )
        self.log_cache_details = bool(self.get_parameter("log_cache_details").value)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "publish_rate_hz <= 0.0 is invalid. Falling back to 10.0 Hz."
            )
            self.publish_rate_hz = 10.0

        if self.signal_timeout_sec <= 0.0:
            self.get_logger().warn(
                "signal_timeout_sec <= 0.0 is invalid. Falling back to 0.75 s."
            )
            self.signal_timeout_sec = 0.75

        qos_spat = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        qos_traffic_light = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.spat_sub = self.create_subscription(
            SpatPacket,
            input_topic,
            self.spat_callback,
            qos_spat,
        )

        self.traffic_light_pub = self.create_publisher(
            TrafficLightGroupArray,
            output_topic,
            qos_traffic_light,
        )

        self.signal_cache: Dict[Tuple[int, int], CachedSignalState] = {}
        self.last_publish_count = 0

        publish_period = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(publish_period, self.publish_cached_states)

        self.get_logger().info("V2I Traffic Light Status Publisher node started.")
        self.get_logger().info(f"Subscribed topic: {input_topic}")
        self.get_logger().info(f"Publishing topic: {output_topic}")
        self.get_logger().info(f"Publish rate: {self.publish_rate_hz:.2f} Hz")
        self.get_logger().info(f"Signal timeout: {self.signal_timeout_sec:.3f} s")
        self.get_logger().info(
            f"Keep last valid on unknown: {self.keep_last_valid_on_unknown}"
        )

    def build_traffic_light_group_id(self, intersection_id: int, signal_group: int) -> int:
        """
        Merge intersection_id and signal_group by concatenation.

        Example:
            intersection_id = 1234
            signal_group = 1
            result = 12341
        """
        return int(f"{intersection_id}{signal_group}")

    def map_event_state_to_element(self, event_state: int) -> TrafficLightElement:
        """
        Map spat_msgs/MovementEvent event_state to
        autoware_perception_msgs/TrafficLightElement.
        """
        element = TrafficLightElement()

        # Default values
        element.shape = TrafficLightElement.CIRCLE
        element.status = TrafficLightElement.SOLID_ON
        element.confidence = 1.0

        if event_state == MovementEvent.STOP_AND_REMAIN:
            element.color = TrafficLightElement.RED

        elif event_state == MovementEvent.PROTECTED_MOVEMENT_ALLOWED:
            element.color = TrafficLightElement.GREEN

        elif event_state == MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED:
            element.color = TrafficLightElement.GREEN

        elif event_state == MovementEvent.PERMISSIVE_CLEARANCE:
            element.color = TrafficLightElement.YELLOW

        else:
            element.color = TrafficLightElement.UNKNOWN
            element.status = TrafficLightElement.SOLID_OFF
            element.confidence = 0.0

        return element

    def is_supported_event_state(self, event_state: int) -> bool:
        return event_state in (
            MovementEvent.STOP_AND_REMAIN,
            MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
            MovementEvent.PERMISSIVE_MOVEMENT_ALLOWED,
        )

    def choose_primary_event(self, events: List[MovementEvent]) -> Optional[MovementEvent]:
        """
        Select a single dominant event for one signal_group.

        Selection rule:
        1) Prefer supported events over unknown events
        2) Prefer events with timing info
        3) If multiple remain, prefer the one with the smallest end time
        4) Fall back to the first event
        """
        if not events:
            return None

        supported_with_timing: List[Tuple[int, MovementEvent]] = []
        supported_without_timing: List[MovementEvent] = []

        for event in events:
            if not self.is_supported_event_state(event.event_state):
                continue

            timing = event.timing
            if timing.has_min_end_time:
                supported_with_timing.append((int(timing.min_end_time), event))
            elif timing.has_max_end_time:
                supported_with_timing.append((int(timing.max_end_time), event))
            else:
                supported_without_timing.append(event)

        if supported_with_timing:
            supported_with_timing.sort(key=lambda item: item[0])
            return supported_with_timing[0][1]

        if supported_without_timing:
            return supported_without_timing[0]

        return events[0]

    def update_cache_entry(
        self,
        intersection_id: int,
        signal_group: int,
        event: MovementEvent,
        now_ns: int,
    ) -> None:
        key = (intersection_id, signal_group)

        timing = event.timing
        new_state = int(event.event_state)

        if not self.is_supported_event_state(new_state) and self.keep_last_valid_on_unknown:
            if key in self.signal_cache:
                if self.log_cache_details:
                    self.get_logger().debug(
                        f"Skipped UNKNOWN update for intersection={intersection_id}, "
                        f"signal_group={signal_group}; keeping last valid state."
                    )
                return

        self.signal_cache[key] = CachedSignalState(
            intersection_id=intersection_id,
            signal_group=signal_group,
            event_state=new_state,
            last_update_ns=now_ns,
            min_end_time=int(timing.min_end_time),
            max_end_time=int(timing.max_end_time),
            has_min_end_time=bool(timing.has_min_end_time),
            has_max_end_time=bool(timing.has_max_end_time),
        )

        if self.log_cache_details:
            self.get_logger().debug(
                f"Cache update: intersection={intersection_id}, "
                f"signal_group={signal_group}, event_state={new_state}, "
                f"cache_size={len(self.signal_cache)}"
            )

    def spat_callback(self, msg: SpatPacket) -> None:
        now_ns = self.get_clock().now().nanoseconds
        updated_count = 0
        skipped_empty_state_count = 0

        for intersection in msg.spat.intersections:
            intersection_id = int(intersection.intersection_id)

            for state in intersection.states:
                signal_group = int(state.signal_group)

                if len(state.events) == 0:
                    skipped_empty_state_count += 1
                    continue

                primary_event = self.choose_primary_event(list(state.events))
                if primary_event is None:
                    skipped_empty_state_count += 1
                    continue

                self.update_cache_entry(
                    intersection_id=intersection_id,
                    signal_group=signal_group,
                    event=primary_event,
                    now_ns=now_ns,
                )
                updated_count += 1

        self.prune_expired_cache(now_ns)

        self.get_logger().debug(
            f"Processed SPaT message: intersections={len(msg.spat.intersections)}, "
            f"updated_states={updated_count}, skipped_empty_states={skipped_empty_state_count}, "
            f"cache_size={len(self.signal_cache)}"
        )

    def prune_expired_cache(self, now_ns: Optional[int] = None) -> int:
        if now_ns is None:
            now_ns = self.get_clock().now().nanoseconds

        timeout_ns = int(self.signal_timeout_sec * 1e9)
        expired_keys = [
            key
            for key, cached in self.signal_cache.items()
            if (now_ns - cached.last_update_ns) > timeout_ns
        ]

        for key in expired_keys:
            cached = self.signal_cache[key]
            if self.log_cache_details:
                self.get_logger().debug(
                    f"Cache timeout: intersection={cached.intersection_id}, "
                    f"signal_group={cached.signal_group}"
                )
            del self.signal_cache[key]

        return len(expired_keys)

    def publish_cached_states(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        expired_count = self.prune_expired_cache(now_ns)

        output_msg = TrafficLightGroupArray()
        output_msg.stamp = self.get_clock().now().to_msg()

        for key in sorted(self.signal_cache.keys()):
            cached = self.signal_cache[key]

            group_msg = TrafficLightGroup()
            group_msg.traffic_light_group_id = self.build_traffic_light_group_id(
                cached.intersection_id,
                cached.signal_group,
            )

            element = self.map_event_state_to_element(cached.event_state)
            group_msg.elements.append(element)

            output_msg.traffic_light_groups.append(group_msg)

        self.traffic_light_pub.publish(output_msg)

        current_count = len(output_msg.traffic_light_groups)
        if current_count != self.last_publish_count or expired_count > 0:
            self.get_logger().debug(
                f"Published {current_count} traffic light groups "
                f"(expired={expired_count}, cache_size={len(self.signal_cache)})."
            )
            self.last_publish_count = current_count


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V2ITrafficLightStatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()