#!/usr/bin/env python3

import math
import uuid
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    Shape,
)
from v2i_sdsm_msgs.msg import SDSM, SDSMDetectedObject


@dataclass
class CachedSdsmObject:
    intersection_key: str
    detected_object: DetectedObject
    last_update_ns: int


class SdsmGroupedMapProjectionNode(Node):
    def __init__(self) -> None:
        super().__init__("sdsm_grouped_map_projection_node")

        self.declare_parameter("input_topic", "/v2i/sdsm/raw")
        self.declare_parameter("output_topic", "/v2i/sdsm/objects")
        self.declare_parameter("target_frame", "map")

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("object_timeout_sec", 0.75)
        self.declare_parameter("use_object_cache", True)

        self.declare_parameter("position_scale_x", 0.1)
        self.declare_parameter("position_scale_y", 0.1)
        self.declare_parameter("position_offset_x", 0.0)
        self.declare_parameter("position_offset_y", 0.0)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)

        self.declare_parameter("vehicle_length_scale", 0.01)
        self.declare_parameter("vehicle_width_scale", 0.01)
        self.declare_parameter("vru_radius_scale", 0.01)
        self.declare_parameter("use_sphere_visualization", False)
        self.declare_parameter("sphere_radius_m", 2.5)

        self.declare_parameter("default_vehicle_height", 1.6)
        self.declare_parameter("default_pedestrian_height", 1.8)
        self.declare_parameter("default_unknown_height", 1.5)

        self.declare_parameter("vehicle_box_z_center_offset", 0.8)
        self.declare_parameter("pedestrian_cylinder_z_center_offset", 0.9)
        self.declare_parameter("unknown_z_center_offset", 0.75)

        self.declare_parameter("vehicle_existence_probability", 0.95)
        self.declare_parameter("pedestrian_existence_probability", 0.95)
        self.declare_parameter("unknown_existence_probability", 0.50)

        self.declare_parameter("use_heading", False)
        self.declare_parameter("default_heading", 45.0)
        self.declare_parameter("heading_scale_rad", 0.0125)

        self.declare_parameter("intersection_keys", [""])

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.object_timeout_sec = float(self.get_parameter("object_timeout_sec").value)
        self.use_object_cache = bool(self.get_parameter("use_object_cache").value)

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
        self.use_sphere_visualization = bool(
            self.get_parameter("use_sphere_visualization").value
        )
        self.sphere_radius_m = float(self.get_parameter("sphere_radius_m").value)

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

        self.intersection_frame_map = self._load_intersection_frame_map()
        self.object_cache: Dict[Tuple[str, object], CachedSdsmObject] = {}
        self.transform_cache: Dict[str, TransformStamped] = {}
        self._last_throttle_times: Dict[str, int] = {}

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(SDSM, input_topic, self.sdsm_callback, qos_in)
        self.objects_pub = self.create_publisher(DetectedObjects, output_topic, qos_out)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "publish_rate_hz <= 0.0 is invalid. Falling back to 10.0 Hz."
            )
            self.publish_rate_hz = 10.0

        if self.object_timeout_sec <= 0.0:
            self.get_logger().warn(
                "object_timeout_sec <= 0.0 is invalid. Falling back to 0.75 s."
            )
            self.object_timeout_sec = 0.75

        if self.sphere_radius_m <= 0.0:
            self.get_logger().warn(
                "sphere_radius_m <= 0.0 is invalid. Falling back to 2.5 m."
            )
            self.sphere_radius_m = 2.5

        if self.use_sphere_visualization:
            self.get_logger().warn(
                "Sphere mode is approximated with equal-diameter CYLINDER shapes "
                "because autoware_perception_msgs/Shape has no SPHERE primitive."
            )

        timer_period = 1.0 / self.publish_rate_hz
        self.publish_timer = self.create_timer(timer_period, self._publish_cached_objects)

        self.get_logger().info("SDSM grouped map projection node started.")
        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing DetectedObjects on: {output_topic}")
        self.get_logger().info(f"Target frame: {self.target_frame}")
        self.get_logger().info(f"Object cache enabled: {self.use_object_cache}")
        self.get_logger().info(
            "Visualization shape mode: sphere"
            if self.use_sphere_visualization
            else "Visualization shape mode: dynamic_box"
        )
        self.get_logger().info(
            f"Loaded {len(self.intersection_frame_map)} intersection frame mappings"
        )

    def _make_header(self, stamp, frame_id: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def _load_intersection_frame_map(self) -> Dict[str, str]:
        intersection_keys = (
            self.get_parameter("intersection_keys")
            .get_parameter_value()
            .string_array_value
        )

        frame_map: Dict[str, str] = {}
        for item in intersection_keys:
            if not item:
                continue

            try:
                key_part, frame_name = item.split(":", 1)
                key_part = key_part.strip()
                frame_name = frame_name.strip()

                if "," in key_part:
                    lat_str, lon_str = key_part.split(",", 1)
                    map_key = f"ref:{int(lat_str.strip())},{int(lon_str.strip())}"
                else:
                    map_key = f"id:{int(key_part)}"

                frame_map[map_key] = frame_name
            except ValueError:
                self.get_logger().warn(f"Invalid intersection_keys entry: {item}")

        return frame_map

    def _get_intersection_key_from_msg(self, msg: SDSM) -> str:
        if hasattr(msg, "intersection_id"):
            try:
                return f"id:{int(msg.intersection_id)}"
            except (TypeError, ValueError):
                pass

        return f"ref:{int(msg.ref_lat)},{int(msg.ref_lon)}"

    def _frame_id_for_sdsm(self, msg: SDSM) -> Optional[str]:
        intersection_key = self._get_intersection_key_from_msg(msg)
        frame_id = self.intersection_frame_map.get(intersection_key)

        if frame_id is None:
            self._warn_throttled(
                f"missing_frame:{intersection_key}",
                f"No frame mapping for {intersection_key}; skipping SDSM objects.",
                5.0,
            )
            return None

        return frame_id

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

    def _yaw_to_quaternion(self, yaw: float) -> Tuple[float, float, float, float]:
        return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _lookup_source_to_target_transform(
        self,
        source_frame: str,
    ) -> Optional[TransformStamped]:
        cached_transform = self.transform_cache.get(source_frame)
        if cached_transform is not None:
            return cached_transform

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
            )
            self.transform_cache[source_frame] = transform
            return transform
        except TransformException as exc:
            self._warn_throttled(
                f"tf_lookup:{source_frame}",
                f"TF lookup failed from {source_frame} to {self.target_frame}: {exc}",
                5.0,
            )
            return None

    def _transform_local_pose_to_target(
        self,
        local_x: float,
        local_y: float,
        local_z: float,
        local_yaw: float,
        transform: TransformStamped,
    ) -> Tuple[float, float, float, float]:
        rotation = transform.transform.rotation
        translation = transform.transform.translation

        qx = float(rotation.x)
        qy = float(rotation.y)
        qz = float(rotation.z)
        qw = float(rotation.w)

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        target_x = (
            (1.0 - 2.0 * (yy + zz)) * local_x
            + 2.0 * (xy - wz) * local_y
            + 2.0 * (xz + wy) * local_z
            + float(translation.x)
        )
        target_y = (
            2.0 * (xy + wz) * local_x
            + (1.0 - 2.0 * (xx + zz)) * local_y
            + 2.0 * (yz - wx) * local_z
            + float(translation.y)
        )
        target_z = (
            2.0 * (xz - wy) * local_x
            + 2.0 * (yz + wx) * local_y
            + (1.0 - 2.0 * (xx + yy)) * local_z
            + float(translation.z)
        )

        tf_yaw = self._quaternion_to_yaw(qx, qy, qz, qw)
        target_yaw = tf_yaw + local_yaw

        return target_x, target_y, target_z, target_yaw

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
        if self.use_sphere_visualization:
            return self._sphere_like_shape_for_object(obj)

        shape = Shape()

        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VEHICLE:
            shape.type = Shape.BOUNDING_BOX
            shape.dimensions.x = max(float(obj.length) * self.vehicle_length_scale, 0.1)
            shape.dimensions.y = max(float(obj.width) * self.vehicle_width_scale, 0.1)
            shape.dimensions.z = self.default_vehicle_height
            return shape, self.vehicle_box_z_center_offset

        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VRU:
            shape.type = Shape.CYLINDER
            radius_m = max(float(obj.vru_radius) * self.vru_radius_scale, 0.1)
            diameter_m = radius_m * 2.0
            shape.dimensions.x = diameter_m
            shape.dimensions.y = diameter_m
            shape.dimensions.z = self.default_pedestrian_height
            return shape, self.pedestrian_cylinder_z_center_offset

        shape.type = Shape.BOUNDING_BOX
        shape.dimensions.x = 0.5
        shape.dimensions.y = 0.5
        shape.dimensions.z = self.default_unknown_height
        return shape, self.unknown_z_center_offset

    def _sphere_like_shape_for_object(self, obj: SDSMDetectedObject) -> Tuple[Shape, float]:
        shape = Shape()
        shape.type = Shape.CYLINDER
        diameter_m = self.sphere_radius_m * 2.0

        shape.dimensions.x = diameter_m
        shape.dimensions.y = diameter_m
        shape.dimensions.z = diameter_m
        return shape, self.sphere_radius_m

    def _existence_probability_for_object(self, obj: SDSMDetectedObject) -> float:
        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VEHICLE:
            return self.vehicle_existence_probability
        if obj.object_type == SDSMDetectedObject.OBJECT_TYPE_VRU:
            return self.pedestrian_existence_probability
        return self.unknown_existence_probability

    def _local_yaw_for_object(self, obj: SDSMDetectedObject) -> Tuple[float, int]:
        if not self.use_heading:
            return math.radians(self.default_heading), DetectedObjectKinematics.UNAVAILABLE

        if obj.heading == 0:
            return 0.0, DetectedObjectKinematics.UNAVAILABLE

        return float(obj.heading) * self.heading_scale_rad, DetectedObjectKinematics.AVAILABLE

    def _build_detected_object(
        self,
        intersection_key: str,
        cache_key: Tuple[str, object],
        obj: SDSMDetectedObject,
        transform: TransformStamped,
    ) -> DetectedObject:
        detected = DetectedObject()
        detected.existence_probability = self._existence_probability_for_object(obj)
        detected.classification.append(self._classification_for_object(obj))

        shape, z_center = self._shape_for_object(obj)
        detected.shape = shape

        local_x, local_y = self._convert_position(obj.position.offset_x, obj.position.offset_y)
        local_yaw, orientation_availability = self._local_yaw_for_object(obj)
        target_x, target_y, target_z, target_yaw = self._transform_local_pose_to_target(
            local_x,
            local_y,
            z_center,
            local_yaw,
            transform,
        )

        kinematics = DetectedObjectKinematics()
        kinematics.pose_with_covariance.pose.position.x = target_x
        kinematics.pose_with_covariance.pose.position.y = target_y
        kinematics.pose_with_covariance.pose.position.z = target_z
        kinematics.has_position_covariance = False
        kinematics.has_twist = False
        kinematics.has_twist_covariance = False
        kinematics.orientation_availability = orientation_availability

        qx, qy, qz, qw = self._yaw_to_quaternion(target_yaw)
        kinematics.pose_with_covariance.pose.orientation.x = qx
        kinematics.pose_with_covariance.pose.orientation.y = qy
        kinematics.pose_with_covariance.pose.orientation.z = qz
        kinematics.pose_with_covariance.pose.orientation.w = qw

        detected.kinematics = kinematics
        self._maybe_assign_uuid(detected, intersection_key, cache_key)
        return detected

    def _make_cache_key(
        self,
        intersection_key: str,
        obj: SDSMDetectedObject,
        local_x: float,
        local_y: float,
    ) -> Tuple[str, object]:
        if hasattr(obj, "object_id"):
            return intersection_key, int(obj.object_id)

        fallback_x = round(local_x, 1)
        fallback_y = round(local_y, 1)
        return intersection_key, (
            int(obj.object_type),
            fallback_x,
            fallback_y,
        )

    def _update_object_cache(self, msg: SDSM, source_frame: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        intersection_key = self._get_intersection_key_from_msg(msg)
        transform = self._lookup_source_to_target_transform(source_frame)
        if transform is None:
            return

        for obj in msg.objects:
            local_x, local_y = self._convert_position(obj.position.offset_x, obj.position.offset_y)
            cache_key = self._make_cache_key(intersection_key, obj, local_x, local_y)
            detected_object = self._build_detected_object(
                intersection_key,
                cache_key,
                obj,
                transform,
            )
            self.object_cache[cache_key] = CachedSdsmObject(
                intersection_key=intersection_key,
                detected_object=detected_object,
                last_update_ns=now_ns,
            )

    def _prune_expired_objects(self, now_ns: Optional[int] = None) -> int:
        if now_ns is None:
            now_ns = self.get_clock().now().nanoseconds

        timeout_ns = int(self.object_timeout_sec * 1e9)
        expired_keys = [
            key
            for key, cached in self.object_cache.items()
            if (now_ns - cached.last_update_ns) > timeout_ns
        ]

        for key in expired_keys:
            del self.object_cache[key]

        return len(expired_keys)

    def _publish_cached_objects(self) -> None:
        if not self.use_object_cache:
            return

        now_ns = self.get_clock().now().nanoseconds
        self._prune_expired_objects(now_ns)

        output_msg = DetectedObjects()
        output_msg.header = self._make_header(
            self.get_clock().now().to_msg(),
            self.target_frame,
        )

        for cached in self.object_cache.values():
            output_msg.objects.append(cached.detected_object)

        self.objects_pub.publish(output_msg)

    def _publish_single_message(self, msg: SDSM, source_frame: str) -> None:
        transform = self._lookup_source_to_target_transform(source_frame)
        if transform is None:
            return

        intersection_key = self._get_intersection_key_from_msg(msg)
        output_msg = DetectedObjects()
        output_msg.header = self._make_header(
            self.get_clock().now().to_msg(),
            self.target_frame,
        )

        for obj in msg.objects:
            local_x, local_y = self._convert_position(obj.position.offset_x, obj.position.offset_y)
            cache_key = self._make_cache_key(intersection_key, obj, local_x, local_y)
            output_msg.objects.append(
                self._build_detected_object(intersection_key, cache_key, obj, transform)
            )

        self.objects_pub.publish(output_msg)

    def _maybe_assign_uuid(
        self,
        detected: DetectedObject,
        intersection_key: str,
        cache_key: Tuple[str, object],
    ) -> None:
        object_id_field = getattr(detected, "object_id", None)
        if object_id_field is None or not hasattr(object_id_field, "uuid"):
            return

        stable_name = f"{intersection_key}:{cache_key[1]}"
        deterministic_uuid = uuid.uuid5(uuid.NAMESPACE_DNS, stable_name)
        object_id_field.uuid = list(deterministic_uuid.bytes)

    def _warn_throttled(self, key: str, message: str, throttle_sec: float) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_throttle_times.get(key)
        if last_ns is None or (now_ns - last_ns) >= int(throttle_sec * 1e9):
            self.get_logger().warn(message)
            self._last_throttle_times[key] = now_ns

    def sdsm_callback(self, msg: SDSM) -> None:
        source_frame = self._frame_id_for_sdsm(msg)
        if source_frame is None:
            return

        if self.use_object_cache:
            if len(msg.objects) == 0:
                return
            self._update_object_cache(msg, source_frame)
            return

        self._publish_single_message(msg, source_frame)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SdsmGroupedMapProjectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
