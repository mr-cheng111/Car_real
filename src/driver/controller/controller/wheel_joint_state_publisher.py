#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import time

from rclpy.node import Node
import rclpy
from ros_robot_controller_msgs.msg import MotorsState
from sensor_msgs.msg import JointState


class WheelJointStatePublisher(Node):
    def __init__(self, name: str) -> None:
        rclpy.init()
        super().__init__(name)

        self.declare_parameter('motor_speed_topic', '/motor_speed')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_wheel_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint_name', 'right_wheel_joint')
        self.declare_parameter('left_motor_id', 2)
        self.declare_parameter('right_motor_id', 1)
        self.declare_parameter('left_wheel_joint_direction', 1.0)
        self.declare_parameter('right_wheel_joint_direction', 1.0)
        self.declare_parameter('wheel_radius', 0.0175)
        self.declare_parameter('motor_speed_unit', 'rpm')
        self.declare_parameter('motor_speed_timeout', 0.2)
        self.declare_parameter('publish_rate', 30.0)

        self.motor_speed_topic = str(self.get_parameter('motor_speed_topic').value)
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.left_wheel_joint_name = str(self.get_parameter('left_wheel_joint_name').value)
        self.right_wheel_joint_name = str(self.get_parameter('right_wheel_joint_name').value)
        self.left_motor_id = int(self.get_parameter('left_motor_id').value)
        self.right_motor_id = int(self.get_parameter('right_motor_id').value)
        self.left_wheel_joint_direction = float(self.get_parameter('left_wheel_joint_direction').value)
        self.right_wheel_joint_direction = float(self.get_parameter('right_wheel_joint_direction').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.motor_speed_unit = str(self.get_parameter('motor_speed_unit').value).lower()
        self.motor_speed_timeout = float(self.get_parameter('motor_speed_timeout').value)
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_time = self.get_clock().now()
        self.last_motor_speed_time = 0.0
        self.warned_invalid_unit = False

        self.joint_state_pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.create_subscription(MotorsState, self.motor_speed_topic, self.motor_speed_callback, 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

        self.get_logger().info(
            'start wheel joint state publisher: motor_speed=%s unit=%s wheel_radius=%.4f'
            % (self.motor_speed_topic, self.motor_speed_unit, self.wheel_radius)
        )

    def speed_to_rad_per_sec(self, speed: float) -> float:
        if self.motor_speed_unit == 'rpm':
            return speed * 2.0 * math.pi / 60.0
        if self.motor_speed_unit == 'rps':
            return speed * 2.0 * math.pi

        if not self.warned_invalid_unit:
            self.get_logger().warn(
                'unsupported motor_speed_unit "%s", treating feedback as rpm' % self.motor_speed_unit
            )
            self.warned_invalid_unit = True
        return speed * 2.0 * math.pi / 60.0

    def motor_speed_callback(self, msg: MotorsState) -> None:
        left_speed = None
        right_speed = None
        for motor in msg.data:
            if motor.id == self.left_motor_id:
                left_speed = float(motor.rps)
            elif motor.id == self.right_motor_id:
                right_speed = float(motor.rps)

        if left_speed is None or right_speed is None:
            return

        self.left_velocity = self.left_wheel_joint_direction * self.speed_to_rad_per_sec(left_speed)
        self.right_velocity = self.right_wheel_joint_direction * self.speed_to_rad_per_sec(right_speed)
        self.last_motor_speed_time = time.time()

    def publish_joint_states(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt < 0.0:
            dt = 0.0

        if self.wheel_radius <= 0.0:
            return

        if self.last_motor_speed_time > 0.0 and time.time() - self.last_motor_speed_time > self.motor_speed_timeout:
            self.left_velocity = 0.0
            self.right_velocity = 0.0

        self.left_wheel_position += self.left_velocity * dt
        self.right_wheel_position += self.right_velocity * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.left_wheel_joint_name, self.right_wheel_joint_name]
        msg.position = [self.left_wheel_position, self.right_wheel_position]
        msg.velocity = [self.left_velocity, self.right_velocity]
        self.joint_state_pub.publish(msg)


def main() -> None:
    node = WheelJointStatePublisher('wheel_joint_state_publisher')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
