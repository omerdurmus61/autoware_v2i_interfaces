#!/usr/bin/env python3

from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tier4_perception_msgs.msg import TrafficLightRoiArray


class TrafficLightRoiCropperNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_roi_cropper_node")

        self.declare_parameter("image_topic", "/sensing/cam2/image_raw")
        self.declare_parameter(
            "roi_topic", "/traffic_light_map_based_detector/output/rois"
        )
        self.declare_parameter("output_dir", "~/tl_hsv_dataset/roi_crops")
        self.declare_parameter("save_every_n_frames", 1)
        self.declare_parameter("min_roi_width", 1)
        self.declare_parameter("min_roi_height", 1)
        self.declare_parameter("sync_queue_size", 20)
        self.declare_parameter("sync_slop_seconds", 0.2)

        self.image_topic = self.get_parameter("image_topic").value
        self.roi_topic = self.get_parameter("roi_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        self.save_every_n_frames = max(
            1, int(self.get_parameter("save_every_n_frames").value)
        )
        self.min_roi_width = max(1, int(self.get_parameter("min_roi_width").value))
        self.min_roi_height = max(1, int(self.get_parameter("min_roi_height").value))
        sync_queue_size = max(1, int(self.get_parameter("sync_queue_size").value))
        sync_slop_seconds = float(self.get_parameter("sync_slop_seconds").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0

        self.debug_publisher = self.create_publisher(
            Image,
            "/traffic_light_roi_cropper/debug/image",
            10,
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
        self.get_logger().info(f"output_dir: {self.output_dir}")
        self.get_logger().info(
            f"save_every_n_frames: {self.save_every_n_frames}, "
            f"min_roi_width: {self.min_roi_width}, "
            f"min_roi_height: {self.min_roi_height}"
        )

    def synced_callback(
        self, image_msg: Image, roi_array_msg: TrafficLightRoiArray
    ) -> None:
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().error(f"Failed to convert image: {error}")
            return

        image_height, image_width = cv_image.shape[:2]
        debug_image = cv_image.copy()
        should_save = (self.frame_count % self.save_every_n_frames) == 0
        timestamp_str = self._format_stamp(image_msg)
        saved_count = 0

        for roi_index, traffic_light_roi in enumerate(roi_array_msg.rois):
            roi = traffic_light_roi.roi

            x0 = max(0, int(roi.x_offset))
            y0 = max(0, int(roi.y_offset))
            x1 = min(image_width, int(roi.x_offset + roi.width))
            y1 = min(image_height, int(roi.y_offset + roi.height))

            clamped_width = x1 - x0
            clamped_height = y1 - y0
            is_valid = (
                clamped_width >= self.min_roi_width
                and clamped_height >= self.min_roi_height
            )

            color = (0, 255, 0) if is_valid else (0, 0, 255)
            if x1 > x0 and y1 > y0:
                cv2.rectangle(debug_image, (x0, y0), (x1 - 1, y1 - 1), color, 2)

            if not is_valid:
                continue

            if not should_save:
                continue

            crop = cv_image[y0:y1, x0:x1]
            if crop.size == 0:
                continue

            file_name = self._build_file_name(
                timestamp_str=timestamp_str,
                roi_index=roi_index,
                traffic_light_id=int(traffic_light_roi.traffic_light_id),
            )
            file_path = self.output_dir / file_name

            if cv2.imwrite(str(file_path), crop):
                saved_count += 1
                self.get_logger().info(f"Saved ROI crop: {file_path}")
            else:
                self.get_logger().error(f"Failed to save ROI crop: {file_path}")

        if self.debug_publisher.get_subscription_count() > 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header = image_msg.header
                self.debug_publisher.publish(debug_msg)
            except CvBridgeError as error:
                self.get_logger().error(f"Failed to publish debug image: {error}")

        if should_save and saved_count == 0 and len(roi_array_msg.rois) > 0:
            self.get_logger().debug("No valid ROI crops were saved for this frame.")

    def _build_file_name(
        self, timestamp_str: str, roi_index: int, traffic_light_id: int
    ) -> str:
        return f"{timestamp_str}_roi_{roi_index:03d}_tlid_{traffic_light_id}.png"

    def _format_stamp(self, image_msg: Image) -> str:
        stamp = image_msg.header.stamp
        return f"{int(stamp.sec):010d}_{int(stamp.nanosec):09d}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightRoiCropperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
