#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from v2i_spat_msgs.msg import SpatPacket, MovementEvent
from autoware_perception_msgs.msg import (
    TrafficLightGroupArray,
    TrafficLightGroup,
    TrafficLightElement,
)


class V2ITrafficLightStatusPublisher(Node):
    def __init__(self) -> None:
        super().__init__('v2i_traffic_light_status_publisher')


        self.declare_parameter("input_topic", "/v2i/spat/raw")
        self.declare_parameter(
            "output_topic",
            "/perception/traffic_light_recognition/traffic_signals",
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)


        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.spat_sub = self.create_subscription(
            SpatPacket,
            '/v2i/spat/raw',
            self.spat_callback,
            qos
        )

        self.traffic_light_pub = self.create_publisher(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            10
        )

        self.get_logger().info('V2I Traffic Light Status Publisher node started.')
        self.get_logger().info("V2I Traffic Light Status Publisher node started.")
        self.get_logger().info(f"Subscribed topic: {input_topic}")
        self.get_logger().info(f"Publishing topic: {output_topic}")

    def build_traffic_light_group_id(self, intersection_id: int, signal_group: int) -> int:
        """
        Merge intersection_id and signal_group by concatenation.

        Example:
            intersection_id = 1234
            signal_group = 1
            result = 12341
        """
        return int(f'{intersection_id}{signal_group}')

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

        else:
            element.color = TrafficLightElement.UNKNOWN
            element.status = TrafficLightElement.SOLID_OFF
            element.confidence = 0.0

        return element

    def spat_callback(self, msg: SpatPacket) -> None:
        output_msg = TrafficLightGroupArray()
        output_msg.stamp = self.get_clock().now().to_msg()

        # msg.spat.intersections -> IntersectionState[]
        for intersection in msg.spat.intersections:
            intersection_id = intersection.intersection_id

            # intersection.states -> MovementState[]
            for state in intersection.states:
                signal_group = state.signal_group

                group_msg = TrafficLightGroup()
                group_msg.traffic_light_group_id = self.build_traffic_light_group_id(
                    intersection_id,
                    signal_group
                )

                # state.events -> MovementEvent[]
                for event in state.events:
                    element = self.map_event_state_to_element(event.event_state)
                    group_msg.elements.append(element)

                output_msg.traffic_light_groups.append(group_msg)

        self.traffic_light_pub.publish(output_msg)

        self.get_logger().debug(
            f'Published {len(output_msg.traffic_light_groups)} traffic light groups.'
        )


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


if __name__ == '__main__':
    main()