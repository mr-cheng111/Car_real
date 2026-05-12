#!/usr/bin/env python3
# encoding: utf-8
import enum
import queue
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

from ros_robot_controller_msgs.msg import MotorState, MotorsState


class PacketControllerState(enum.IntEnum):
    PACKET_CONTROLLER_STATE_STARTBYTE1 = 0
    PACKET_CONTROLLER_STATE_STARTBYTE2 = 1
    PACKET_CONTROLLER_STATE_LENGTH = 2
    PACKET_CONTROLLER_STATE_FUNCTION = 3
    PACKET_CONTROLLER_STATE_DATA = 5
    PACKET_CONTROLLER_STATE_CHECKSUM = 6


class PacketFunction(enum.IntEnum):
    PACKET_FUNC_SYS = 0
    PACKET_FUNC_LED = 1
    PACKET_FUNC_BUZZER = 2
    PACKET_FUNC_MOTOR = 3
    PACKET_FUNC_SPEAKER = 4
    PACKET_FUNC_WKUP = 5
    PACKET_FUNC_KEY = 6
    PACKET_FUNC_HOUSEHOLD = 7
    PACKET_FUNC_GP2Y = 8
    PACKET_FUNC_LEARN = 9
    PACKET_FUNC_MOTOR_SPEED = 10
    PACKET_FUNC_NONE = 11


def checksum_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            crc &= 0xFF
    return crc & 0xFF


class MotorSpeedSerialReader:
    def __init__(
        self,
        device='/dev/ttyS0',
        baudrate=115200,
        timeout=0.02,
        left_offset=0,
        right_offset=4,
    ):
        self.port = Serial(device, baudrate, timeout=timeout)
        self.left_offset = left_offset
        self.right_offset = right_offset
        self.enable_recv = False
        self.frame = []
        self.recv_count = 0
        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
        self.motor_speed_queue = queue.Queue(maxsize=1)
        self.thread = None

    def start(self):
        if self.enable_recv:
            return
        self.enable_recv = True
        self.thread = threading.Thread(target=self.recv_task, daemon=True)
        self.thread.start()

    def close(self):
        self.enable_recv = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.port.is_open:
            self.port.close()

    def get_motor_speed(self):
        try:
            data = self.motor_speed_queue.get(block=False)
        except queue.Empty:
            return None

        min_size = max(self.left_offset, self.right_offset) + 2
        if len(data) < min_size:
            return None

        left_speed = struct.unpack_from('<h', data, self.left_offset)[0]
        right_speed = struct.unpack_from('<h', data, self.right_offset)[0]
        return left_speed, right_speed

    def _put_latest_motor_speed(self, data):
        try:
            self.motor_speed_queue.put_nowait(data)
        except queue.Full:
            try:
                self.motor_speed_queue.get_nowait()
            except queue.Empty:
                pass
            self.motor_speed_queue.put_nowait(data)

    def recv_task(self):
        while self.enable_recv:
            recv_data = self.port.read()
            if not recv_data:
                continue

            for dat in recv_data:
                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1:
                    if dat == 0xAA:
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2
                    continue

                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2:
                    if dat == 0x55:
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION
                    else:
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                    continue

                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION:
                    if dat < int(PacketFunction.PACKET_FUNC_NONE):
                        self.frame = [dat, 0]
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH
                    else:
                        self.frame = []
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                    continue

                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH:
                    self.frame[1] = dat
                    self.recv_count = 0
                    if dat == 0:
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                    else:
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_DATA
                    continue

                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_DATA:
                    self.frame.append(dat)
                    self.recv_count += 1
                    if self.recv_count >= self.frame[1]:
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                    continue

                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM:
                    if checksum_crc8(bytes(self.frame)) == dat:
                        func = PacketFunction(self.frame[0])
                        data = bytes(self.frame[2:])
                        if func == PacketFunction.PACKET_FUNC_MOTOR_SPEED:
                            self._put_latest_motor_speed(data)
                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1


class MotorSpeedReaderNode(Node):
    def __init__(self):
        super().__init__('motor_speed_reader')
        self.declare_parameter('device', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.02)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('raw_topic', '/motor_speed/raw_array')
        self.declare_parameter('motors_topic', '/motor_speed')
        self.declare_parameter('left_motor_id', 2)
        self.declare_parameter('right_motor_id', 1)
        self.declare_parameter('left_speed_offset', 0)
        self.declare_parameter('right_speed_offset', 4)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('left_speed_sign', -1.0)
        self.declare_parameter('right_speed_sign', 1.0)

        self.device = self.get_parameter('device').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.raw_topic = self.get_parameter('raw_topic').value
        self.motors_topic = self.get_parameter('motors_topic').value
        self.left_motor_id = int(self.get_parameter('left_motor_id').value)
        self.right_motor_id = int(self.get_parameter('right_motor_id').value)
        self.left_speed_offset = int(self.get_parameter('left_speed_offset').value)
        self.right_speed_offset = int(self.get_parameter('right_speed_offset').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.left_speed_sign = float(self.get_parameter('left_speed_sign').value)
        self.right_speed_sign = float(self.get_parameter('right_speed_sign').value)

        self.raw_pub = self.create_publisher(Int16MultiArray, self.raw_topic, 10)
        self.motors_pub = self.create_publisher(MotorsState, self.motors_topic, 10)

        try:
            self.reader = MotorSpeedSerialReader(
                device=self.device,
                baudrate=self.baudrate,
                timeout=self.serial_timeout,
                left_offset=self.left_speed_offset,
                right_offset=self.right_speed_offset,
            )
            self.reader.start()
        except SerialException as exc:
            self.get_logger().fatal('open serial failed: %s' % exc)
            raise

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self.publish_motor_speed)
        self.last_report_time = time.monotonic()
        self.get_logger().info(
            'start: device=%s baudrate=%d publish_rate=%.1f raw_topic=%s motors_topic=%s'
            % (self.device, self.baudrate, self.publish_rate, self.raw_topic, self.motors_topic)
        )

    def publish_motor_speed(self):
        speeds = self.reader.get_motor_speed()
        if speeds is None:
            now = time.monotonic()
            if now - self.last_report_time > 5.0:
                self.get_logger().warn('no motor speed frame received yet')
                self.last_report_time = now
            return

        left_speed, right_speed = speeds

        raw_msg = Int16MultiArray()
        raw_msg.layout.dim.append(MultiArrayDimension(label='wheel', size=2, stride=2))
        raw_msg.layout.data_offset = 0
        raw_msg.data = [int(left_speed), int(right_speed)]
        self.raw_pub.publish(raw_msg)

        motors_msg = MotorsState()
        left_motor = MotorState()
        left_motor.id = self.left_motor_id
        left_motor.rps = float(left_speed) * self.speed_scale * self.left_speed_sign
        right_motor = MotorState()
        right_motor.id = self.right_motor_id
        right_motor.rps = float(right_speed) * self.speed_scale * self.right_speed_sign
        motors_msg.data = [left_motor, right_motor]
        self.motors_pub.publish(motors_msg)

    def destroy_node(self):
        if hasattr(self, 'reader'):
            self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
