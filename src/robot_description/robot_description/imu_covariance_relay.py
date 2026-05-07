import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceRelay(Node):
    def __init__(self) -> None:
        super().__init__('imu_covariance_relay')

        self.subscription = self.create_subscription(
            Imu,
            'imu/raw',
            self.handle_imu,
            10,
        )
        self.publisher = self.create_publisher(Imu, 'imu', 10)

        gyro_var = 0.0012 ** 2
        accel_var = 0.008 ** 2

        self.orientation_covariance = [-1.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0]
        self.angular_velocity_covariance = [gyro_var, 0.0, 0.0,
                                            0.0, gyro_var, 0.0,
                                            0.0, 0.0, gyro_var]
        self.linear_acceleration_covariance = [accel_var, 0.0, 0.0,
                                               0.0, accel_var, 0.0,
                                               0.0, 0.0, accel_var]

    def handle_imu(self, msg: Imu) -> None:
        output = copy.deepcopy(msg)
        output.orientation_covariance = self.orientation_covariance
        output.angular_velocity_covariance = self.angular_velocity_covariance
        output.linear_acceleration_covariance = self.linear_acceleration_covariance
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuCovarianceRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

