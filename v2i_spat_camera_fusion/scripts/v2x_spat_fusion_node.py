#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from autoware_perception_msgs.msg import TrafficLightGroupArray


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
        self.declare_parameter("publish_rate_hz", 10.0)

        camera_input_topic = str(self.get_parameter("camera_input_topic").value)
        spat_input_topic = str(self.get_parameter("spat_input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

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
        self.publish_timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_empty_output,
        )

        self.get_logger().warn(
            "Hybrid method is not implemented yet. Fusion node will be added later."
        )
        self.get_logger().info(f"camera_input_topic: {camera_input_topic}")
        self.get_logger().info(f"spat_input_topic: {spat_input_topic}")
        self.get_logger().info(f"output_topic: {output_topic}")
        self.get_logger().info(f"publish_rate_hz: {publish_rate_hz:.2f}")

    def camera_callback(self, msg: TrafficLightGroupArray) -> None:
        _ = msg

    def spat_callback(self, msg: TrafficLightGroupArray) -> None:
        _ = msg

    def publish_empty_output(self) -> None:
        output_msg = TrafficLightGroupArray()
        output_msg.stamp = self.get_clock().now().to_msg()
        self.output_pub.publish(output_msg)


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
