#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelJointStatePublisher(Node):
    def __init__(self, name: str) -> None:
        rclpy.init()
        super().__init__(name)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_wheel_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint_name', 'right_wheel_joint')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_track', 0.2948)
        self.declare_parameter('publish_rate', 30.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.left_wheel_joint_name = str(self.get_parameter('left_wheel_joint_name').value)
        self.right_wheel_joint_name = str(self.get_parameter('right_wheel_joint_name').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_time = self.get_clock().now()

        self.joint_state_pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

        self.get_logger().info(
            'start wheel joint state publisher: cmd_vel=%s wheel_radius=%.4f wheel_track=%.4f'
            % (self.cmd_vel_topic, self.wheel_radius, self.wheel_track)
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.linear_x = float(msg.linear.x)
        self.angular_z = float(msg.angular.z)

    def publish_joint_states(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt < 0.0:
            dt = 0.0

        if self.wheel_radius <= 0.0:
            return

        # 差速底盘左右轮线速度公式：
        # v_l = v - ω * L / 2
        # v_r = v + ω * L / 2
        # 再由滚动约束 v = r * \dot{θ} 得：
        # \dot{θ}_l = v_l / r, \dot{θ}_r = v_r / r
        left_linear = self.linear_x - self.angular_z * self.wheel_track / 2.0
        right_linear = self.linear_x + self.angular_z * self.wheel_track / 2.0

        left_velocity = left_linear / self.wheel_radius
        right_velocity = right_linear / self.wheel_radius

        self.left_wheel_position += left_velocity * dt
        self.right_wheel_position += right_velocity * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.left_wheel_joint_name, self.right_wheel_joint_name]
        msg.position = [self.left_wheel_position, self.right_wheel_position]
        msg.velocity = [left_velocity, right_velocity]
        self.joint_state_pub.publish(msg)


def main() -> None:
    node = WheelJointStatePublisher('wheel_joint_state_publisher')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
