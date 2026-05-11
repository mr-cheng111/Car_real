#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def normalize_quaternion(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def quaternion_yaw_only(x, y, z, w):
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class ImuOrientationTf(Node):
    def __init__(self):
        super().__init__("imu_orientation_tf")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_footprint")
        self.declare_parameter("invert_orientation", False)
        self.declare_parameter("yaw_only", False)

        self.imu_topic = self.get_parameter("imu_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.invert_orientation = as_bool(self.get_parameter("invert_orientation").value)
        self.yaw_only = as_bool(self.get_parameter("yaw_only").value)

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, self.imu_topic, self.handle_imu, 10)
        self.get_logger().info(
            f"broadcasting IMU orientation TF: {self.parent_frame} -> "
            f"{self.child_frame}, imu_topic={self.imu_topic}, "
            f"invert_orientation={self.invert_orientation}, yaw_only={self.yaw_only}"
        )

    def handle_imu(self, msg):
        if msg.orientation_covariance[0] < 0.0:
            self.get_logger().warn(
                "IMU orientation covariance is -1, so orientation is marked unavailable.",
                throttle_duration_sec=5.0,
            )
            return

        qx, qy, qz, qw = normalize_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        if self.invert_orientation:
            qx, qy, qz = -qx, -qy, -qz
        if self.yaw_only:
            qx, qy, qz, qw = quaternion_yaw_only(qx, qy, qz, qw)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = ImuOrientationTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
