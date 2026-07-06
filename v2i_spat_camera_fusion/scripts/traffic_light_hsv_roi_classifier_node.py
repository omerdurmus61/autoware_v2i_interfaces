#!/usr/bin/env python3

from copy import deepcopy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from autoware_perception_msgs.msg import (
    TrafficLightGroup,
    TrafficLightGroupArray,
    TrafficLightElement,
)
from tier4_perception_msgs.msg import TrafficLightRoiArray
import yaml


class TrafficLightHsvRoiClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_hsv_roi_classifier_node")

        self.declare_parameter("image_topic", "/sensing/cam2/image_raw")
        self.declare_parameter(
            "roi_topic", "/traffic_light_map_based_detector/output/rois"
        )
        self.declare_parameter(
            "output_image_topic", "/traffic_light_hsv_roi_classifier/debug/image"
        )
        self.declare_parameter(
            "output_signals_topic", "/traffic_light_classifier/output/traffic_signals"
        )
        self.declare_parameter("traffic_light_id_map_path", "")
        self.declare_parameter("min_roi_width", 1)
        self.declare_parameter("min_roi_height", 1)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("sync_queue_size", 20)
        self.declare_parameter("sync_slop_seconds", 0.2)
        self.declare_parameter("saturation_threshold", 60)
        self.declare_parameter("value_threshold", 80)
        self.declare_parameter("bright_quantile", 0.8)
        self.declare_parameter("saturation_quantile", 0.5)
        self.declare_parameter("fallback_bright_quantile", 0.97)
        self.declare_parameter("fallback_saturation_quantile", 0.7)
        self.declare_parameter("center_weight_factor", 20.0)
        self.declare_parameter("green_hue_min", 35.0)
        self.declare_parameter("green_hue_max", 95.0)
        self.declare_parameter("red_hue_low_max", 15.0)
        self.declare_parameter("red_hue_high_min", 165.0)
        self.declare_parameter("yellow_hue_min", 16.0)
        self.declare_parameter("yellow_hue_max", 34.0)
        self.declare_parameter("g_over_r_split_1", 1.03)
        self.declare_parameter("g_over_r_split_2", 1.08)
        self.declare_parameter("s_mean_split_1", 110.50)
        self.declare_parameter("s_mean_split_2", 125.61)
        self.declare_parameter("s_mean_split_3", 125.27)
        self.declare_parameter("s_mean_split_4", 77.70)
        self.declare_parameter("r_minus_g_split", 48.41)
        self.declare_parameter("yellow_ratio_split_1", 0.78)
        self.declare_parameter("yellow_ratio_split_2", 0.44)
        self.declare_parameter("green_ratio_weight", 0.6)
        self.declare_parameter("green_g_over_r_weight", 0.4)
        self.declare_parameter("red_ratio_weight", 0.6)
        self.declare_parameter("red_r_minus_g_weight", 0.4)
        self.declare_parameter("green_g_over_r_norm", 2.0)
        self.declare_parameter("red_r_minus_g_bias", 255.0)
        self.declare_parameter("red_r_minus_g_norm", 510.0)
        self.declare_parameter("label_score_floor", 0.55)
        self.declare_parameter("opposite_score_ceiling", 0.45)

        self.image_topic = self.get_parameter("image_topic").value
        self.roi_topic = self.get_parameter("roi_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value
        self.output_signals_topic = self.get_parameter("output_signals_topic").value
        self.traffic_light_id_map_path = Path(
            self.get_parameter("traffic_light_id_map_path").value
        ).expanduser()
        self.min_roi_width = max(1, int(self.get_parameter("min_roi_width").value))
        self.min_roi_height = max(1, int(self.get_parameter("min_roi_height").value))
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        sync_queue_size = max(1, int(self.get_parameter("sync_queue_size").value))
        sync_slop_seconds = float(self.get_parameter("sync_slop_seconds").value)
        self.saturation_threshold = int(
            self.get_parameter("saturation_threshold").value
        )
        self.value_threshold = int(self.get_parameter("value_threshold").value)
        self.bright_quantile = float(self.get_parameter("bright_quantile").value)
        self.saturation_quantile = float(
            self.get_parameter("saturation_quantile").value
        )
        self.fallback_bright_quantile = float(
            self.get_parameter("fallback_bright_quantile").value
        )
        self.fallback_saturation_quantile = float(
            self.get_parameter("fallback_saturation_quantile").value
        )
        self.center_weight_factor = float(
            self.get_parameter("center_weight_factor").value
        )
        self.green_hue_min = float(self.get_parameter("green_hue_min").value)
        self.green_hue_max = float(self.get_parameter("green_hue_max").value)
        self.red_hue_low_max = float(self.get_parameter("red_hue_low_max").value)
        self.red_hue_high_min = float(self.get_parameter("red_hue_high_min").value)
        self.yellow_hue_min = float(self.get_parameter("yellow_hue_min").value)
        self.yellow_hue_max = float(self.get_parameter("yellow_hue_max").value)
        self.g_over_r_split_1 = float(self.get_parameter("g_over_r_split_1").value)
        self.g_over_r_split_2 = float(self.get_parameter("g_over_r_split_2").value)
        self.s_mean_split_1 = float(self.get_parameter("s_mean_split_1").value)
        self.s_mean_split_2 = float(self.get_parameter("s_mean_split_2").value)
        self.s_mean_split_3 = float(self.get_parameter("s_mean_split_3").value)
        self.s_mean_split_4 = float(self.get_parameter("s_mean_split_4").value)
        self.r_minus_g_split = float(self.get_parameter("r_minus_g_split").value)
        self.yellow_ratio_split_1 = float(
            self.get_parameter("yellow_ratio_split_1").value
        )
        self.yellow_ratio_split_2 = float(
            self.get_parameter("yellow_ratio_split_2").value
        )
        self.green_ratio_weight = float(
            self.get_parameter("green_ratio_weight").value
        )
        self.green_g_over_r_weight = float(
            self.get_parameter("green_g_over_r_weight").value
        )
        self.red_ratio_weight = float(self.get_parameter("red_ratio_weight").value)
        self.red_r_minus_g_weight = float(
            self.get_parameter("red_r_minus_g_weight").value
        )
        self.green_g_over_r_norm = float(
            self.get_parameter("green_g_over_r_norm").value
        )
        self.red_r_minus_g_bias = float(
            self.get_parameter("red_r_minus_g_bias").value
        )
        self.red_r_minus_g_norm = float(
            self.get_parameter("red_r_minus_g_norm").value
        )
        self.label_score_floor = float(self.get_parameter("label_score_floor").value)
        self.opposite_score_ceiling = float(
            self.get_parameter("opposite_score_ceiling").value
        )

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "publish_rate_hz <= 0.0 is invalid. Falling back to 10.0 Hz."
            )
            self.publish_rate_hz = 10.0

        self.bridge = CvBridge()
        self.traffic_light_id_map = self._load_traffic_light_id_map()
        self.cached_output_signals = TrafficLightGroupArray()
        self.debug_publisher = self.create_publisher(
            Image,
            self.output_image_topic,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.traffic_signal_publisher = self.create_publisher(
            TrafficLightGroupArray,
            self.output_signals_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_cached_signals,
        )

        self.image_sub = Subscriber(
            self,
            Image,
            self.image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.roi_sub = Subscriber(
            self,
            TrafficLightRoiArray,
            self.roi_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.roi_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_seconds,
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"roi_topic: {self.roi_topic}")
        self.get_logger().info(f"output_image_topic: {self.output_image_topic}")
        self.get_logger().info(f"output_signals_topic: {self.output_signals_topic}")
        self.get_logger().info(f"publish_rate_hz: {self.publish_rate_hz:.2f} Hz")
        self.get_logger().info(
            f"loaded {len(self.traffic_light_id_map)} traffic light ID mappings"
        )

    def publish_cached_signals(self) -> None:
        output_signals = TrafficLightGroupArray()
        output_signals.stamp = self.get_clock().now().to_msg()
        output_signals.traffic_light_groups = deepcopy(
            self.cached_output_signals.traffic_light_groups
        )
        self.traffic_signal_publisher.publish(output_signals)

    def synced_callback(
        self, image_msg: Image, roi_array_msg: TrafficLightRoiArray
    ) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().error(f"Failed to convert image: {error}")
            return

        image_height, image_width = cv_image.shape[:2]
        debug_image = cv_image.copy()
        output_signals = TrafficLightGroupArray()
        output_signals.stamp = image_msg.header.stamp

        for traffic_light_roi in roi_array_msg.rois:
            roi = traffic_light_roi.roi
            x0 = max(0, int(roi.x_offset))
            y0 = max(0, int(roi.y_offset))
            x1 = min(image_width, int(roi.x_offset + roi.width))
            y1 = min(image_height, int(roi.y_offset + roi.height))

            clamped_width = x1 - x0
            clamped_height = y1 - y0
            if (
                clamped_width < self.min_roi_width
                or clamped_height < self.min_roi_height
                or x1 <= x0
                or y1 <= y0
            ):
                continue

            roi_image = cv_image[y0:y1, x0:x1]
            label, green_score, red_score, coverage = self.classify_hsv(roi_image)
            color = self.label_to_color(label)
            mapped_traffic_light_id = self._map_traffic_light_id(
                int(traffic_light_roi.traffic_light_id)
            )
            output_signals.traffic_light_groups.append(
                self._build_traffic_light_group(
                    traffic_light_group_id=mapped_traffic_light_id,
                    label=label,
                    green_score=green_score,
                    red_score=red_score,
                )
            )

            cv2.rectangle(debug_image, (x0, y0), (x1 - 1, y1 - 1), color, 2)

            text = (
                f"{label.upper()}  g:{green_score:.2f}  "
                f"r:{red_score:.2f}  id:{mapped_traffic_light_id}"
            )
            text_origin_y = y0 - 10 if y0 > 28 else min(image_height - 10, y1 + 24)
            self._draw_label_text(
                debug_image,
                text,
                (x0, text_origin_y),
                color,
                font_scale=0.82,
                thickness=2,
            )

        self.cached_output_signals = output_signals

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = image_msg.header
            self.debug_publisher.publish(debug_msg)
        except CvBridgeError as error:
            self.get_logger().error(f"Failed to publish debug image: {error}")

    def build_pre_filter_mask(self, hsv: np.ndarray) -> np.ndarray:
        s_channel, v_channel = hsv[:, :, 1], hsv[:, :, 2]
        mask = (s_channel > self.saturation_threshold) & (
            v_channel > self.value_threshold
        )
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        return mask.astype(bool)

    def classify_hsv(self, bgr_image: np.ndarray) -> tuple[str, float, float, float]:
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = self._build_signal_mask(hsv)
        if not np.any(mask):
            return "unknown", 0.0, 0.0, 0.0

        feature_vector = self._extract_feature_vector(bgr_image, hsv, mask)
        label, green_score, red_score = self._classify_feature_vector(feature_vector)
        coverage = float(mask.mean())

        return label, green_score, red_score, coverage

    def _build_signal_mask(self, hsv: np.ndarray) -> np.ndarray:
        base_mask = self.build_pre_filter_mask(hsv)
        if not np.any(base_mask):
            return self._build_fallback_mask(hsv)

        value_channel = hsv[:, :, 2]
        saturation_channel = hsv[:, :, 1]
        bright_threshold = max(
            self.value_threshold,
            int(np.quantile(value_channel[base_mask], self.bright_quantile)),
        )
        saturation_threshold = max(
            self.saturation_threshold,
            int(np.quantile(saturation_channel[base_mask], self.saturation_quantile)),
        )
        signal_mask = (
            base_mask
            & (value_channel >= bright_threshold)
            & (saturation_channel >= saturation_threshold)
        )
        return signal_mask if np.any(signal_mask) else base_mask

    def _build_fallback_mask(self, hsv: np.ndarray) -> np.ndarray:
        saturation_channel = hsv[:, :, 1]
        value_channel = hsv[:, :, 2]
        bright_threshold = int(
            np.quantile(value_channel, self.fallback_bright_quantile)
        )
        saturation_threshold = int(
            np.quantile(saturation_channel, self.fallback_saturation_quantile)
        )
        fallback_mask = (value_channel >= bright_threshold) & (
            saturation_channel >= saturation_threshold
        )
        if np.any(fallback_mask):
            return fallback_mask

        max_value = int(value_channel.max())
        return value_channel >= max_value

    def _extract_feature_vector(
        self, bgr_image: np.ndarray, hsv: np.ndarray, mask: np.ndarray
    ) -> np.ndarray:
        y_coords, x_coords = np.where(mask)
        image_height, image_width = mask.shape
        center_x = (image_width - 1) / 2.0
        center_y = (image_height - 1) / 2.0
        distance = ((x_coords - center_x) / max(image_width, 1)) ** 2 + (
            (y_coords - center_y) / max(image_height, 1)
        ) ** 2
        weights = 1.0 / (1.0 + self.center_weight_factor * distance)

        hue = hsv[:, :, 0][y_coords, x_coords].astype(np.float32)
        saturation = hsv[:, :, 1][y_coords, x_coords].astype(np.float32)
        value = hsv[:, :, 2][y_coords, x_coords].astype(np.float32)
        bgr_pixels = bgr_image[y_coords, x_coords].astype(np.float32)
        green = bgr_pixels[:, 1]
        red = bgr_pixels[:, 2]

        hue_histogram = np.zeros(18, dtype=np.float32)
        hue_bins = np.clip((hue // 10).astype(np.int32), 0, 17)
        for hue_bin, weight in zip(hue_bins, weights):
            hue_histogram[hue_bin] += weight
        if hue_histogram.sum() > 0.0:
            hue_histogram /= hue_histogram.sum()

        green_ratio = np.average(
            ((hue >= self.green_hue_min) & (hue <= self.green_hue_max)).astype(
                np.float32
            ),
            weights=weights,
        )
        red_ratio = np.average(
            ((hue <= self.red_hue_low_max) | (hue >= self.red_hue_high_min)).astype(
                np.float32
            ),
            weights=weights,
        )
        yellow_ratio = np.average(
            ((hue >= self.yellow_hue_min) & (hue <= self.yellow_hue_max)).astype(
                np.float32
            ),
            weights=weights,
        )

        extra_features = np.array(
            [
                green_ratio,
                red_ratio,
                yellow_ratio,
                np.average(green - red, weights=weights),
                np.average(red - green, weights=weights),
                np.average(green / (red + 1.0), weights=weights),
                np.average(red / (green + 1.0), weights=weights),
                np.average(saturation, weights=weights),
                np.average(value, weights=weights),
            ],
            dtype=np.float32,
        )
        return np.concatenate([hue_histogram, extra_features]).astype(np.float32)

    def _classify_feature_vector(
        self, feature_vector: np.ndarray
    ) -> tuple[str, float, float]:
        green_ratio = float(feature_vector[18])
        red_ratio = float(feature_vector[19])
        yellow_ratio = float(feature_vector[20])
        r_minus_g = float(feature_vector[22])
        g_over_r = float(feature_vector[23])
        s_mean = float(feature_vector[25])

        if g_over_r <= self.g_over_r_split_1:
            if s_mean <= self.s_mean_split_1:
                label = "green" if r_minus_g <= self.r_minus_g_split else "red"
            elif s_mean <= self.s_mean_split_2:
                if yellow_ratio <= self.yellow_ratio_split_1:
                    label = "red" if s_mean <= self.s_mean_split_3 else "green"
                else:
                    label = "green"
            else:
                label = "red"
        else:
            if s_mean <= self.s_mean_split_4:
                label = "red"
            elif g_over_r <= self.g_over_r_split_2:
                label = "red" if yellow_ratio <= self.yellow_ratio_split_2 else "green"
            else:
                label = "green"

        green_score = float(
            np.clip(
                self.green_ratio_weight * green_ratio
                + self.green_g_over_r_weight * g_over_r / self.green_g_over_r_norm,
                0.0,
                1.0,
            )
        )
        red_score = float(
            np.clip(
                self.red_ratio_weight * red_ratio
                + self.red_r_minus_g_weight
                * (r_minus_g + self.red_r_minus_g_bias)
                / self.red_r_minus_g_norm,
                0.0,
                1.0,
            )
        )

        if label == "green":
            green_score = max(green_score, self.label_score_floor)
            red_score = min(red_score, self.opposite_score_ceiling)
        else:
            red_score = max(red_score, self.label_score_floor)
            green_score = min(green_score, self.opposite_score_ceiling)

        return label, green_score, red_score

    def label_to_color(self, label: str) -> tuple[int, int, int]:
        if label == "green":
            return (0, 255, 0)
        if label == "red":
            return (0, 0, 255)
        return (0, 255, 255)

    def _draw_label_text(
        self,
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
        color: tuple[int, int, int],
        font_scale: float,
        thickness: int,
    ) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, font_scale, thickness
        )
        x0, y0 = origin
        top_left = (max(0, x0 - 6), max(0, y0 - text_height - 10))
        bottom_right = (
            min(image.shape[1] - 1, x0 + text_width + 8),
            min(image.shape[0] - 1, y0 + baseline + 6),
        )
        cv2.rectangle(image, top_left, bottom_right, (0, 0, 0), -1)
        cv2.putText(
            image,
            text,
            origin,
            font,
            font_scale,
            (255, 255, 255),
            thickness + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            origin,
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _build_traffic_light_group(
        self,
        traffic_light_group_id: int,
        label: str,
        green_score: float,
        red_score: float,
    ) -> TrafficLightGroup:
        signal = TrafficLightGroup()
        signal.traffic_light_group_id = traffic_light_group_id

        element = TrafficLightElement()
        element.shape = TrafficLightElement.CIRCLE
        element.status = TrafficLightElement.SOLID_ON

        if label == "green":
            element.color = TrafficLightElement.GREEN
            element.confidence = float(green_score)
        elif label == "red":
            element.color = TrafficLightElement.RED
            element.confidence = float(red_score)
        else:
            element.color = TrafficLightElement.UNKNOWN
            element.shape = TrafficLightElement.UNKNOWN
            element.status = TrafficLightElement.UNKNOWN
            element.confidence = 0.0

        signal.elements.append(element)
        return signal

    def _load_traffic_light_id_map(self) -> dict[int, int]:
        if not str(self.traffic_light_id_map_path):
            return {}
        if not self.traffic_light_id_map_path.exists():
            self.get_logger().warn(
                f"traffic_light_id_map_path does not exist: {self.traffic_light_id_map_path}"
            )
            return {}

        with self.traffic_light_id_map_path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        raw_map = data.get("traffic_light_id_map", {})
        return {int(key): int(value) for key, value in raw_map.items()}

    def _map_traffic_light_id(self, bulbs_id: int) -> int:
        return self.traffic_light_id_map.get(bulbs_id, bulbs_id)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightHsvRoiClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
