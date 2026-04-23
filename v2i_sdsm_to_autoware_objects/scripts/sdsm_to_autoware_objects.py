#!/usr/bin/env python3

import math
from typing import Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

from v2i_sdsm_msgs.msg import SDSM, SDSMDetectedObject
from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjects,
    ObjectClassification,
    DetectedObjectKinematics,
    Shape,
)

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class SdsmToAutowareObjects(Node):
    def __init__(self) -> None:
        super().__init__("sdsm_to_autoware_objects")

        # Topics / frame
        self.declare_parameter("input_topic", "/v2i/sdsm/raw")
        self.declare_parameter("output_topic", "/v2i/sdsm/objects")
        self.declare_parameter("frame_id", "v2i_intersection_14867")
        self.declare_parameter("intersection_keys", [""])

        # Position conversion
        self.declare_parameter("position_scale_x", 0.1)
        self.declare_parameter("position_scale_y", 0.1)
        self.declare_parameter("position_offset_x", 0.0)
        self.declare_parameter("position_offset_y", 0.0)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        # Size conversion
        self.declare_parameter("vehicle_length_scale", 0.01)
        self.declare_parameter("vehicle_width_scale", 0.01)
        self.declare_parameter("vru_radius_scale", 0.01)

        # Default dimensions / placement
        self.declare_parameter("default_vehicle_height", 1.6)
        self.declare_parameter("default_pedestrian_height", 1.8)
        self.declare_parameter("default_unknown_height", 1.5)

        self.declare_parameter("vehicle_box_z_center_offset", 0.8)
        self.declare_parameter("pedestrian_cylinder_z_center_offset", 0.9)
        self.declare_parameter("unknown_z_center_offset", 0.75)

        # Classification / existence probability
        self.declare_parameter("vehicle_existence_probability", 0.95)
        self.declare_parameter("pedestrian_existence_probability", 0.95)
        self.declare_parameter("unknown_existence_probability", 0.50)

        # Heading usage
        self.declare_parameter("use_heading", False)
        self.declare_parameter("default_heading", 45)
        self.declare_parameter("heading_scale_rad", 0.0125)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.intersection_frame_map = self._load_intersection_frame_map()

        self.position_scale_x = float(self.get_parameter("position_scale_x").value)
        self.position_scale_y = float(self.get_parameter("position_scale_y").value)
        self.position_offset_x = float(self.get_parameter("position_offset_x").value)
        self.position_offset_y = float(self.get_parameter("position_offset_y").value)
        self.swap_xy = bool(self.get_parameter("swap_xy").value)
        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)

        self.vehicle_length_scale = float(self.get_parameter("vehicle_length_scale").value)
        self.vehicle_width_scale = float(self.get_parameter("vehicle_width_scale").value)
        self.vru_radius_scale = float(self.get_parameter("vru_radius_scale").value)

        self.default_vehicle_height = float(self.get_parameter("default_vehicle_height").value)
        self.default_pedestrian_height = float(self.get_parameter("default_pedestrian_height").value)
        self.default_unknown_height = float(self.get_parameter("default_unknown_height").value)

        self.vehicle_box_z_center_offset = float(self.get_parameter("vehicle_box_z_center_offset").value)
        self.pedestrian_cylinder_z_center_offset = float(
            self.get_parameter("pedestrian_cylinder_z_center_offset").value
        )
        self.unknown_z_center_offset = float(self.get_parameter("unknown_z_center_offset").value)

        self.vehicle_existence_probability = float(
            self.get_parameter("vehicle_existence_probability").value
        )
        self.pedestrian_existence_probability = float(
            self.get_parameter("pedestrian_existence_probability").value
        )
        self.unknown_existence_probability = float(
            self.get_parameter("unknown_existence_probability").value
        )

        self.use_heading = bool(self.get_parameter("use_heading").value)
        self.default_heading = float(self.get_parameter("default_heading").value)
        self.heading_scale_rad = float(self.get_parameter("heading_scale_rad").value)

        # QoS (broadcast-like)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )


        self.sub = self.create_subscription(
            SDSM,
            input_topic,
            self.sdsm_callback,
            qos,
        )

        self.objects_pub = self.create_publisher(
            DetectedObjects,
            output_topic,
            10,
        )

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing DetectedObjects on: {output_topic}")
        self.get_logger().info(f"Output frame_id: {self.frame_id}")

    def _make_header(self, stamp, frame_id: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def _convert_position(self, raw_x: int, raw_y: int) -> Tuple[float, float]:
        x = raw_x * self.position_scale_x + self.position_offset_x
        y = raw_y * self.position_scale_y + self.position_offset_y

        if self.swap_xy:
            x, y = y, x
        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y

        return x, y

    def _yaw_to_quaternion(self, yaw: float):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return qx, qy, qz, qw

    def _classification_for_object(self, obj: SDSMDetectedObject) -> ObjectClassification:
        cls = ObjectClassification()

        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VEHICLE:
            cls.label = ObjectClassification.CAR
            cls.probability = 0.95
        elif obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VRU:
            if obj.vru_basic_type == "aPEDESTRIAN":
                cls.label = ObjectClassification.PEDESTRIAN
                cls.probability = 0.95
            else:
                cls.label = ObjectClassification.UNKNOWN
                cls.probability = 0.50
        else:
            cls.label = ObjectClassification.UNKNOWN
            cls.probability = 0.50

        return cls

    def _shape_for_object(self, obj: SDSMDetectedObject) -> Tuple[Shape, float]:
        shape = Shape()

        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VEHICLE:
            shape.type = Shape.BOUNDING_BOX

            length_m = max(float(obj.length) * self.vehicle_length_scale, 0.1)
            width_m = max(float(obj.width) * self.vehicle_width_scale, 0.1)
            height_m = self.default_vehicle_height

            shape.dimensions.x = length_m
            shape.dimensions.y = width_m
            shape.dimensions.z = height_m

            return shape, self.vehicle_box_z_center_offset

        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VRU:
            shape.type = Shape.CYLINDER

            radius_m = max(float(obj.vru_radius) * self.vru_radius_scale, 0.1)
            diameter_m = radius_m * 2.0
            height_m = self.default_pedestrian_height

            shape.dimensions.x = diameter_m
            shape.dimensions.y = diameter_m
            shape.dimensions.z = height_m

            return shape, self.pedestrian_cylinder_z_center_offset

        shape.type = Shape.BOUNDING_BOX
        shape.dimensions.x = 0.5
        shape.dimensions.y = 0.5
        shape.dimensions.z = self.default_unknown_height

        return shape, self.unknown_z_center_offset

    def _existence_probability_for_object(self, obj: SDSMDetectedObject) -> float:
        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VEHICLE:
            return self.vehicle_existence_probability
        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VRU:
            return self.pedestrian_existence_probability
        return self.unknown_existence_probability

    def _fill_orientation(self, obj: SDSMDetectedObject, detected: DetectedObject) -> None:
        kinematics = detected.kinematics

        if not self.use_heading:
            yaw = math.radians(self.default_heading)

            kinematics.pose_with_covariance.pose.orientation.x = 0.0
            kinematics.pose_with_covariance.pose.orientation.y = 0.0
            kinematics.pose_with_covariance.pose.orientation.z = math.sin(yaw / 2.0)
            kinematics.pose_with_covariance.pose.orientation.w = math.cos(yaw / 2.0)
            return

        if obj.heading == 0:
            kinematics.orientation_availability = DetectedObjectKinematics.UNAVAILABLE
            kinematics.pose_with_covariance.pose.orientation.x = 0.0
            kinematics.pose_with_covariance.pose.orientation.y = 0.0
            kinematics.pose_with_covariance.pose.orientation.z = 0.0
            kinematics.pose_with_covariance.pose.orientation.w = 1.0
            return

        yaw = float(obj.heading) * self.heading_scale_rad
        qx, qy, qz, qw = self._yaw_to_quaternion(yaw)

        kinematics.pose_with_covariance.pose.orientation.x = qx
        kinematics.pose_with_covariance.pose.orientation.y = qy
        kinematics.pose_with_covariance.pose.orientation.z = qz
        kinematics.pose_with_covariance.pose.orientation.w = qw
        kinematics.orientation_availability = DetectedObjectKinematics.AVAILABLE

    def _build_detected_object(self, obj: SDSMDetectedObject) -> DetectedObject:
        detected = DetectedObject()
        detected.existence_probability = self._existence_probability_for_object(obj)
        detected.classification.append(self._classification_for_object(obj))

        shape, z_center = self._shape_for_object(obj)
        detected.shape = shape

        x_m, y_m = self._convert_position(obj.position.offset_x, obj.position.offset_y)

        kinematics = DetectedObjectKinematics()
        kinematics.pose_with_covariance.pose.position.x = x_m
        kinematics.pose_with_covariance.pose.position.y = y_m
        kinematics.pose_with_covariance.pose.position.z = z_center

        kinematics.has_position_covariance = False
        kinematics.has_twist = False
        kinematics.has_twist_covariance = False

        detected.kinematics = kinematics
        self._fill_orientation(obj, detected)

        return detected

    def sdsm_callback(self, msg: SDSM) -> None:
        frame_id = self._frame_id_for_sdsm(msg)

        detected_objects_msg = DetectedObjects()
        detected_objects_msg.header = self._make_header(
            self.get_clock().now().to_msg(),
            frame_id,
        )

        for obj in msg.objects:
            detected_objects_msg.objects.append(self._build_detected_object(obj))

        self.objects_pub.publish(detected_objects_msg)

    def _load_intersection_frame_map(self):
        intersection_keys = (
            self.get_parameter("intersection_keys")
            .get_parameter_value()
            .string_array_value
        )

        frame_map = {}

        for item in intersection_keys:
            if not item:
                continue

            try:
                key_part, frame_name = item.split(":")
                lat_str, lon_str = key_part.split(",")

                ref_lat = int(lat_str.strip())
                ref_lon = int(lon_str.strip())
                frame_map[(ref_lat, ref_lon)] = frame_name.strip()

            except ValueError:
                self.get_logger().warn(
                    f"Invalid intersection_keys entry: {item}"
        )

        self.get_logger().info(
            f"Loaded {len(frame_map)} intersection frame mappings"
        )

        return frame_map


    def _frame_id_for_sdsm(self, msg: SDSM) -> str:
        key = (int(msg.ref_lat), int(msg.ref_lon))

        frame_id = self.intersection_frame_map.get(key)

        if frame_id is None:
            self.get_logger().warn(
                f"Unknown intersection ref_lat={msg.ref_lat}, ref_lon={msg.ref_lon}. "
                f"Using default frame_id={self.frame_id}"
            )
            return self.frame_id

        return frame_id

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SdsmToAutowareObjects()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()