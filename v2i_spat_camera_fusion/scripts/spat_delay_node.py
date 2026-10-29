#!/usr/bin/env python3

"""Delay SPaT group arrays without changing their contents or timestamps."""

from collections import deque
from copy import deepcopy
import math

import rclpy
from autoware_perception_msgs.msg import TrafficLightGroupArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class SpatDelayNode(Node):
    def __init__(self):
        super().__init__("spat_delay_node")
        self.declare_parameter("input_topic", "/v2i/spat/traffic_signals")
        self.declare_parameter("output_topic", "/v2i/spat/traffic_signals_delayed")
        self.declare_parameter("delay_ms", 0.0)
        self.declare_parameter("timer_period_ms", 5.0)
        self.delay_ms = float(self.get_parameter("delay_ms").value)
        period_ms = float(self.get_parameter("timer_period_ms").value)
        if not math.isfinite(self.delay_ms) or self.delay_ms < 0.0:
            raise ValueError("delay_ms must be finite and nonnegative")
        if not math.isfinite(period_ms) or period_ms <= 0.0:
            raise ValueError("timer_period_ms must be finite and positive")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.pending = deque()
        self.last_clock_ns = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(TrafficLightGroupArray, output_topic, qos)
        self.subscription = self.create_subscription(
            TrafficLightGroupArray, input_topic, self.receive, qos
        )
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            raise ValueError("input_topic and output_topic must resolve to different topics")
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.timer = self.create_timer(period_ms / 1000.0, self.release_due)
        self.get_logger().info(
            f"SPaT delay: {input_topic} -> {output_topic}, delay_ms={self.delay_ms}"
        )

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "delay_ms":
                if (isinstance(parameter.value, bool)
                        or not isinstance(parameter.value, (int, float))
                        or not math.isfinite(parameter.value) or parameter.value < 0.0):
                    return SetParametersResult(
                        successful=False, reason="delay_ms must be finite and nonnegative"
                    )
            elif parameter.name in {"input_topic", "output_topic", "timer_period_ms"}:
                return SetParametersResult(
                    successful=False, reason=f"Restart the node to change {parameter.name}"
                )
        # Read committed delay_ms in receive(); validation has no side effects.
        return SetParametersResult(successful=True)

    def clock_now(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.pending.clear()
            self.get_logger().warn("ROS clock moved backwards; cleared pending SPaT messages")
        self.last_clock_ns = now_ns
        return now_ns

    def receive(self, msg):
        now_ns = self.clock_now()
        delay_ms = float(self.get_parameter("delay_ms").value)
        due_ns = now_ns + int(delay_ms * 1_000_000)
        # Preserve FIFO when delay is reduced while older messages are pending.
        if self.pending:
            due_ns = max(due_ns, self.pending[-1][0])
        self.pending.append((due_ns, deepcopy(msg)))
        self.release_due()

    def release_due(self):
        now_ns = self.clock_now()
        while self.pending and self.pending[0][0] <= now_ns:
            _, msg = self.pending.popleft()
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpatDelayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
