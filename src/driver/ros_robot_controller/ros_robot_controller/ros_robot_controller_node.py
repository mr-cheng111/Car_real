#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/08/28
# stm32 ros2 package
import math
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import UInt16, Bool
from geometry_msgs.msg import Twist
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.srv import GetBusServoState, GetPWMServoState
from ros_robot_controller_msgs.msg import ButtonState, BuzzerState, LedState, BusServoState, MotorState, MotorsState, SetBusServoState, ServosPosition, SetPWMServoState, Sbus, OLEDState


def _clip(value, low, high):
    return max(low, min(high, value))

class RosRobotController(Node):
    gravity = 9.80665
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        # 声明参数
        self.declare_parameter('device', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('init_finish', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_track', 0.2948)
        self.declare_parameter('wheel_diameter', 0.035)
        self.declare_parameter('motor_gain', 1.0)
        self.declare_parameter('max_motor_speed', 100.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('motor_speed_topic', '/motor_speed')
        self.declare_parameter('motor_speed_raw_topic', '/motor_speed/raw')
        self.declare_parameter('motor_speed_scale', 1.0)
        self.declare_parameter('motor_speed_unit', 'rpm')
        self.declare_parameter('left_feedback_sign',  1.0)
        self.declare_parameter('right_feedback_sign',-1.0)
        self.declare_parameter('left_speed_offset', 0)
        self.declare_parameter('right_speed_offset', 4)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('enable_speed_closed_loop', True)
        self.declare_parameter('speed_kp', 6.0)
        self.declare_parameter('speed_ki', 1.0)
        self.declare_parameter('speed_kd', 0.0)
        self.declare_parameter('speed_integral_limit', 50.0)
        self.device = str(self.get_parameter('device').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.IMU_FRAME = self.get_parameter('imu_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.motor_gain = float(self.get_parameter('motor_gain').value)
        self.max_motor_speed = abs(float(self.get_parameter('max_motor_speed').value))
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.publish_imu = bool(self.get_parameter('publish_imu').value)
        self.motor_speed_topic = str(self.get_parameter('motor_speed_topic').value)
        self.motor_speed_raw_topic = str(self.get_parameter('motor_speed_raw_topic').value)
        self.motor_speed_scale = float(self.get_parameter('motor_speed_scale').value)
        self.motor_speed_unit = str(self.get_parameter('motor_speed_unit').value).lower()
        self.left_feedback_sign = float(self.get_parameter('left_feedback_sign').value)
        self.right_feedback_sign = float(self.get_parameter('right_feedback_sign').value)
        self.left_speed_offset = int(self.get_parameter('left_speed_offset').value)
        self.right_speed_offset = int(self.get_parameter('right_speed_offset').value)
        self.control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.enable_speed_closed_loop = bool(self.get_parameter('enable_speed_closed_loop').value)
        self.speed_kp = float(self.get_parameter('speed_kp').value)
        self.speed_ki = float(self.get_parameter('speed_ki').value)
        self.speed_kd = float(self.get_parameter('speed_kd').value)
        self.speed_integral_limit = abs(float(self.get_parameter('speed_integral_limit').value))
        self.board = Board(device=self.device, baudrate=self.baudrate)
        self.board.enable_reception()
        self.running = True
        self.last_cmd_vel_time = time.monotonic()
        self.last_control_time = time.monotonic()
        self.last_motor_speed_time = 0.0
        self.has_cmd_vel = False
        self.motors_stopped = True
        self.target_left_rps = 0.0
        self.target_right_rps = 0.0
        self.measured_left_rps = 0.0
        self.measured_right_rps = 0.0
        self.left_error_integral = 0.0
        self.right_error_integral = 0.0
        self.last_left_error = 0.0
        self.last_right_error = 0.0
        self.warned_invalid_motor_speed_unit = False

        self.imu_pub = self.create_publisher(Imu, '~/imu_raw', 1)
        self.joy_pub = self.create_publisher(Joy, '~/joy', 1)
        self.sbus_pub = self.create_publisher(Sbus, '~/sbus', 1)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)
        self.motor_speed_pub = self.create_publisher(MotorsState, self.motor_speed_topic, 10)
        self.motor_speed_raw_pub = self.create_publisher(MotorsState, self.motor_speed_raw_topic, 10)
        self.create_subscription(LedState, '~/set_led', self.set_led_state, 5)
        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_state, 5)
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_state, 5)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception, 1)
        self.create_subscription(SetBusServoState, '~/bus_servo/set_state', self.set_bus_servo_state, 10)
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)
        self.create_subscription(SetPWMServoState, '~/pwm_servo/set_state', self.set_pwm_servo_state, 10)
        self.create_service(GetBusServoState, '~/bus_servo/get_state', self.get_bus_servo_state)
        self.create_service(GetPWMServoState, '~/pwm_servo/get_state', self.get_pwm_servo_state)

        self.board.pwm_servo_set_offset(1, 0)
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        self.clock = self.get_clock()
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_timer(0.1, self.cmd_vel_watchdog)
        self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info(
            '\033[1;32mstart: device=%s baudrate=%d cmd_vel_topic=%s wheel_track=%.3f wheel_diameter=%.3f motor_gain=%.2f closed_loop=%s publish_imu=%s\033[0m'
            % (self.device, self.baudrate, self.cmd_vel_topic, self.wheel_track, self.wheel_diameter, self.motor_gain, self.enable_speed_closed_loop, self.publish_imu)
        )

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pub_callback(self):
        while self.running:
            if self.enable_reception:
                self.pub_button_data(self.button_pub)
                self.pub_joy_data(self.joy_pub)
                if self.publish_imu:
                    self.pub_imu_data(self.imu_pub)
                self.pub_sbus_data(self.sbus_pub)
                self.pub_battery_data(self.battery_pub)
                self.pub_motor_speed_data()
                time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

    def enable_reception(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_reception ' + str(msg.data)))
        self.enable_reception = msg.data
        self.board.enable_reception(msg.data)

    def set_led_state(self, msg):
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    def set_buzzer_state(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def _linear_speed_to_motor_speed(self, speed):
        if self.wheel_diameter <= 0.0:
            return 0.0
        return speed / (math.pi * self.wheel_diameter) * self.motor_gain

    def _linear_speed_to_rps(self, speed):
        if self.wheel_diameter <= 0.0:
            return 0.0
        # 滚动约束: v = pi * D * n，因此轮子转速 n = v / (pi * D)，单位 rps。
        return speed / (math.pi * self.wheel_diameter)

    def _feedback_speed_to_rps(self, speed):
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

    def _stop_motors(self):
        self.board.set_motor_speed([[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]])
        self.motors_stopped = True
        self.target_left_rps = 0.0
        self.target_right_rps = 0.0
        self.left_error_integral = 0.0
        self.right_error_integral = 0.0
        self.last_left_error = 0.0
        self.last_right_error = 0.0

    def _pid_motor_speed(self, target_rps, measured_rps, integral, last_error, dt):
        feedforward = target_rps * self.motor_gain
        if not self.enable_speed_closed_loop or self.last_motor_speed_time <= 0.0:
            return feedforward, integral, target_rps - measured_rps

        error = target_rps - measured_rps
        integral += error * dt
        integral = _clip(integral, -self.speed_integral_limit, self.speed_integral_limit)
        derivative = (error - last_error) / dt if dt > 0.0 else 0.0
        correction = self.speed_kp * error + self.speed_ki * integral + self.speed_kd * derivative
        return feedforward + correction, integral, error

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_time = time.monotonic()
        self.has_cmd_vel = True

        linear_speed = float(msg.linear.x)
        angular_speed = float(msg.angular.z)

        # ROS 约定: linear.x > 0 为车体 +X 前进，angular.z > 0 为逆时针左转。
        # 差速逆解公式来源于车体中心速度:
        # v_left = v + omega * L / 2, v_right = v - omega * L / 2。
        # 当前硬件极性: ID1=右轮，正号为前进；ID2=左轮，正号为后退。
        left_linear = linear_speed + angular_speed * self.wheel_track / 2.0
        right_linear = linear_speed - angular_speed * self.wheel_track / 2.0

        self.target_left_rps = self._linear_speed_to_rps(left_linear)
        self.target_right_rps = self._linear_speed_to_rps(right_linear)
        self.motors_stopped = linear_speed == 0.0 and angular_speed == 0.0

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_control_time
        self.last_control_time = now
        if dt <= 0.0:
            return

        if not self.has_cmd_vel or self.motors_stopped:
            return

        left_motor_forward, self.left_error_integral, self.last_left_error = self._pid_motor_speed(
            self.target_left_rps,
            self.measured_left_rps,
            self.left_error_integral,
            self.last_left_error,
            dt,
        )
        right_motor_forward, self.right_error_integral, self.last_right_error = self._pid_motor_speed(
            self.target_right_rps,
            self.measured_right_rps,
            self.right_error_integral,
            self.last_right_error,
            dt,
        )

        right_motor = _clip(right_motor_forward, -self.max_motor_speed, self.max_motor_speed)
        left_motor = _clip(-left_motor_forward, -self.max_motor_speed, self.max_motor_speed)
        self.board.set_motor_speed([[1, right_motor], [2, left_motor]])

    def cmd_vel_watchdog(self):
        if not self.has_cmd_vel or self.motors_stopped:
            return
        if time.monotonic() - self.last_cmd_vel_time > self.cmd_vel_timeout:
            self.get_logger().warn('cmd_vel timeout, stopping motors')
            self._stop_motors()

    def pub_motor_speed_data(self):
        data = self.board.get_motor_speed(self.left_speed_offset, self.right_speed_offset)
        if data is None:
            return

        raw_left, raw_right = data
        self.last_motor_speed_time = time.monotonic()

        raw_msg = MotorsState()
        raw_left_msg = MotorState()
        raw_left_msg.id = 2
        raw_left_msg.rps = float(raw_left)
        raw_right_msg = MotorState()
        raw_right_msg.id = 1
        raw_right_msg.rps = float(raw_right)
        raw_msg.data = [raw_left_msg, raw_right_msg]
        self.motor_speed_raw_pub.publish(raw_msg)

        # 反馈统一到 ROS 车体坐标: 左右轮向车体 +X 滚动都为正。
        left_speed = float(raw_left) * self.motor_speed_scale * self.left_feedback_sign
        right_speed = float(raw_right) * self.motor_speed_scale * self.right_feedback_sign
        self.measured_left_rps = self._feedback_speed_to_rps(left_speed)
        self.measured_right_rps = self._feedback_speed_to_rps(right_speed)

        speed_msg = MotorsState()
        left_msg = MotorState()
        left_msg.id = 2
        left_msg.rps = left_speed
        right_msg = MotorState()
        right_msg.id = 1
        right_msg.rps = right_speed
        speed_msg.data = [left_msg, right_msg]
        self.motor_speed_pub.publish(speed_msg)

    def set_oled_state(self, msg):
        self.board.set_oled_text(int(msg.index), msg.text)

    def set_pwm_servo_state(self, msg):
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.extend([[i.id[0], i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])

        if data != []:
            self.board.pwm_servo_set_position(msg.duration, data)

    def get_pwm_servo_state(self, msg):
        states = []
        for i in msg.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            states.append(data)
        return [True, states]

    def set_bus_servo_position(self, msg):
        data = []
        for i in msg.position:
            data.extend([[i.id, i.position]])
        if data:
            self.board.bus_servo_set_position(msg.duration, data)

    def set_bus_servo_state(self, msg):
        data = []
        servo_id = []
        for i in msg.state:
            if i.present_id:
                if i.present_id[0]:
                    if i.target_id:
                        if i.target_id[0]:
                            self.board.bus_servo_set_id(i.present_id[1], i.target_id[1])
                    if i.position:
                        if i.position[0]:
                            data.extend([[i.present_id[1], i.position[1]]])
                    if i.offset:
                        if i.offset[0]:
                            self.board.bus_servo_set_offset(i.present_id[1], i.offset[1])
                    if i.position_limit:
                        if i.position_limit[0]:
                            self.board.bus_servo_set_angle_limit(i.present_id[1], i.position_limit[1:])
                    if i.voltage_limit:
                        if i.voltage_limit[0]:
                            self.board.bus_servo_set_vin_limit(i.present_id[1], i.voltage_limit[1:])
                    if i.max_temperature_limit:
                        if i.max_temperature_limit[0]:
                            self.board.bus_servo_set_temp_limit(i.present_id[1], i.max_temperature_limit[1])
                    if i.enable_torque:
                        if i.enable_torque[0]:
                            self.board.bus_servo_enable_torque(i.present_id[1], i.enable_torque[1])
                    if i.save_offset:
                        if i.save_offset[0]:
                            self.board.bus_servo_save_offset(i.present_id[1])
                    if i.stop:
                        if i.stop[0]:
                            servo_id.append(i.present_id[1])
        if data != []:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_id != []:    
            self.board.bus_servo_stop(servo_id)

    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None:
                    i.id = state[0]
                    data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            if i.get_voltage:
                state = self.board.bus_servo_read_voltage(i.id)
                if state is not None:
                    data.voltage = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None:
                    data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None:
                    data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None:
                    data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None:
                    data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque(i.id)
                if state is not None:
                    data.enable_torque = state
            states.append(data)
        response.state = states
        response.success = True
        return response

    def pub_battery_data(self, pub):
        data = self.board.get_battery()
        if data is not None:
            msg = UInt16()
            msg.data = data
            pub.publish(msg)

    def pub_button_data(self, pub):
        data = self.board.get_button()
        if data is not None:
            msg = ButtonState()
            msg.id = data[0]
            msg.state = data[1]
            pub.publish(msg)

    def pub_joy_data(self, pub):
        data = self.board.get_gamepad()
        if data is not None:
            msg = Joy()
            msg.axes = data[0]
            msg.buttons = data[1]
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    def pub_sbus_data(self, pub):
        data = self.board.get_sbus()
        if data is not None:
            msg = Sbus()
            msg.channel = data
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    def pub_imu_data(self, pub):
        data = self.board.get_imu()
        if data is not None:
            ax, ay, az, gx, gy, gz = data
            msg = Imu()
            msg.header.frame_id = self.IMU_FRAME
            msg.header.stamp = self.clock.now().to_msg()

            msg.orientation.w = 0.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0

            msg.linear_acceleration.x = ax * self.gravity
            msg.linear_acceleration.y = ay * self.gravity
            msg.linear_acceleration.z = az * self.gravity

            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)

            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.linear_acceleration_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.004]
            pub.publish(msg)

def main():
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == '__main__':
    main()
