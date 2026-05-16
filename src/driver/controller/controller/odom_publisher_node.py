#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseWithCovarianceStamped, TransformStamped
from ros_robot_controller_msgs.msg import MotorsState

ODOM_POSE_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                             0, 1e-3, 1e-9, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]))

ODOM_TWIST_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-9]))

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

class Controller(Node):
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.x = 0.0
        self.y = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.pose_yaw = 0
        self.last_time = None
        self.current_time = None
        self.last_motor_speed_time = 0.0
        signal.signal(signal.SIGINT, self.shutdown)

        # 声明参数
        self.declare_parameter('pub_odom_topic', True)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('linear_correction_factor', 1.00)
        self.declare_parameter('angular_correction_factor', 1.00)
        self.declare_parameter('machine_type', os.environ.get('MACHINE_TYPE', 'rk3588'))
        self.declare_parameter('use_wheel_speed_feedback', True)
        self.declare_parameter('motor_speed_topic', '/motor_speed')
        self.declare_parameter('wheel_diameter', 0.07)
        self.declare_parameter('wheel_track', 0.2948)
        self.declare_parameter('motor_speed_unit', 'rpm')
        self.declare_parameter('left_motor_id', 2)
        self.declare_parameter('right_motor_id', 1)
        self.declare_parameter('motor_speed_timeout', 0.2)
        self.declare_parameter('wheel_linear_direction', -1.0)
        
        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        
        self.linear_factor = self.get_parameter('linear_correction_factor').value
        self.angular_factor = self.get_parameter('angular_correction_factor').value
        self.use_wheel_speed_feedback = bool(self.get_parameter('use_wheel_speed_feedback').value)
        self.motor_speed_topic = str(self.get_parameter('motor_speed_topic').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.motor_speed_unit = str(self.get_parameter('motor_speed_unit').value).lower()
        self.left_motor_id = int(self.get_parameter('left_motor_id').value)
        self.right_motor_id = int(self.get_parameter('right_motor_id').value)
        self.motor_speed_timeout = float(self.get_parameter('motor_speed_timeout').value)
        self.wheel_linear_direction = float(self.get_parameter('wheel_linear_direction').value)
        self.machine_type = str(self.get_parameter('machine_type').value)
        self.warned_invalid_motor_speed_unit = False

        self.clock = self.get_clock() 
        if self.pub_odom_topic:
            # self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)  # 定义TF变换广播者
            # self.odom_trans = TransformStamped()
            # self.odom_trans.header.frame_id = self.odom_frame_id
            # self.odom_trans.child_frame_id = self.base_frame_id
            
            self.odom = Odometry()
            self.odom.header.frame_id = self.odom_frame_id
            self.odom.child_frame_id = self.base_frame_id
            
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            
            self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)
            self.dt = 1.0/50.0

            threading.Thread(target=self.cal_odom_fun, daemon=True).start()
        self.get_logger().info('\033[1;32m%f %f\033[0m' % (self.linear_factor, self.angular_factor))
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 1)
        self.create_subscription(Pose2D, 'set_odom', self.set_odom, 1)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(MotorsState, self.motor_speed_topic, self.motor_speed_callback, 10)
        self.create_service(Trigger, 'controller/load_calibrate_param', self.load_calibrate_param)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def speed_to_rps(self, speed):
        if self.motor_speed_unit == 'rpm':
            return speed / 60.0
        if self.motor_speed_unit == 'rps':
            return speed

        if not self.warned_invalid_motor_speed_unit:
            self.get_logger().warn(
                'unsupported motor_speed_unit "%s", treating feedback as rpm' % self.motor_speed_unit
            )
            self.warned_invalid_motor_speed_unit = True
        return speed / 60.0

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        # 不直接调用rclpy.shutdown()，让launch系统处理
        # rclpy.shutdown()在launch环境中会由launch系统调用，避免重复
        try:
            self.destroy_node()
        except:
            pass

    def load_calibrate_param(self, request, response):
        self.linear_factor = self.get_parameter('~linear_correction_factor').value or 1.00
        self.angular_factor = self.get_parameter('~angular_correction_factor').value or 1.00
        self.get_logger().info('\033[1;32m%s\033[0m' % 'load_calibrate_param')

        response.success = True
        return response

    def set_odom(self, msg):
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id
        
        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom.pose.pose.position.x = msg.x
        self.odom.pose.pose.position.y = msg.y
        self.pose_yaw = msg.theta
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = self.clock().now().to_msg()
        pose.pose.pose = self.odom.pose.pose
        pose.pose.covariance = ODOM_POSE_COVARIANCE
        self.pose_pub.publish(pose)

    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        if not self.use_wheel_speed_feedback:
            self.linear_x = self.cmd_linear_x
            self.linear_y = 0.0
            self.angular_z = self.cmd_angular_z

    def motor_speed_callback(self, msg):
        left_speed = None
        right_speed = None
        for motor in msg.data:
            if motor.id == self.left_motor_id:
                left_speed = float(motor.rps)
            elif motor.id == self.right_motor_id:
                right_speed = float(motor.rps)

        if left_speed is None or right_speed is None:
            return
        if self.wheel_diameter <= 0.0 or self.wheel_track <= 0.0:
            return

        # 轮速反解算:
        # 滚动约束 v_wheel = pi * D * n，其中 n 为 rps；反馈可配置为 rpm/rps。
        # 差速模型 v_l = v + omega * L / 2, v_r = v - omega * L / 2，
        # 因此 v = (v_l + v_r) / 2, omega = (v_l - v_r) / L。
        left_rps = self.speed_to_rps(left_speed)
        right_rps = self.speed_to_rps(right_speed)
        left_linear = math.pi * self.wheel_diameter * left_rps
        right_linear = math.pi * self.wheel_diameter * right_rps
        self.linear_x = self.wheel_linear_direction * (left_linear + right_linear) / 2.0
        self.linear_y = 0.0
        self.angular_z = (left_linear - right_linear) / self.wheel_track
        self.last_motor_speed_time = time.time()

    def cal_odom_fun(self):
        while True:
            self.current_time = time.time()
            if self.last_time is None:
                self.dt = 0.0
            else:
                # 计算时间间隔
                self.dt = self.current_time - self.last_time
            self.odom.header.stamp = self.clock.now().to_msg()

            if self.use_wheel_speed_feedback and self.last_motor_speed_time > 0.0:
                if self.current_time - self.last_motor_speed_time > self.motor_speed_timeout:
                    self.linear_x = 0.0
                    self.linear_y = 0.0
                    self.angular_z = 0.0

            self.x += math.cos(self.pose_yaw)*self.linear_x*self.dt - math.sin(self.pose_yaw)*self.linear_y*self.dt
            self.y += math.sin(self.pose_yaw)*self.linear_x*self.dt + math.cos(self.pose_yaw)*self.linear_y*self.dt

            self.odom.pose.pose.position.x = self.linear_factor*self.x
            self.odom.pose.pose.position.y = self.linear_factor*self.y
            self.odom.pose.pose.position.z = 0.0

            self.pose_yaw += self.angular_factor*self.angular_z*self.dt

            self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
            self.odom.twist.twist.linear.x = self.linear_x
            self.odom.twist.twist.linear.y = self.linear_y
            self.odom.twist.twist.angular.z = self.angular_z

            # self.odom_trans.header.stamp = self.clock.now().to_msg()
            # self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
            # self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
            # self.odom_trans.transform.translation.z = 0.0
            # self.odom_trans.transform.rotation = self.odom.pose.pose.orientation

            # 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
            # 如果velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
            if self.linear_x == 0 and self.linear_y == 0 and self.angular_z == 0:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
            else:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE

            # self.odom_broadcaster.sendTransform(self.odom_trans)
            self.odom_pub.publish(self.odom)
            self.last_time = self.current_time
            time.sleep(0.02)

def main():
    node = Controller('odom_publisher')
    rclpy.spin(node)  # 循环等待ROS2退出

if __name__ == "__main__":
    main()
