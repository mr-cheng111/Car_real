#!/usr/bin/env python3
# encoding: utf-8
import time
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller.ros_robot_controller_sdk import Board, PacketFunction

class DiffDriveTestNode(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 声明 ROS 参数
        self.declare_parameter('device', '/dev/ttyS0')   # 默认测试串口，按需在运行中可更改
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_track', 0.20)      # 左右轮间距
        self.declare_parameter('max_speed', 100.0)       # 预设的系数最大值
        self.declare_parameter('watchdog_timeout', 2.0)
        
        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.max_speed = self.get_parameter('max_speed').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value

        self.get_logger().info(f"Initializing Board on {device} with {baudrate} baud...")
        
        try:
            self.board = Board(device=device, baudrate=baudrate)
            # 注意: 如果你的底层需要开启接收线程，可以解开下述注释
            # self.board.enable_reception()
            self.get_logger().info("Board initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize board: {e}")
            raise e

        # 订阅标准 cmd_vel 话题
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.last_cmd_time = time.time()
        self.is_stopped = True

        # 设置看门狗定时器 (0.1秒执行一次)
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info("diff_drive_test_node has started. Listening on /cmd_vel...")

    def send_motor_like_test_py(self, speed_right, speed_left):
        data = [0x01, 2]
        data.extend(struct.pack("<Bf", 1, float(speed_right)))
        data.extend(struct.pack("<Bf", 2, float(speed_left)))
        self.board.buf_write(PacketFunction.PACKET_FUNC_MOTOR, data)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        self.is_stopped = False

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 1. 差速逆解: 计算左右轮理论线速度
        # 约定 angular_z > 0 为左转(CCW)。
        # 对当前硬件映射需使用该符号组合，避免左右转反向。
        vl = linear_x + (angular_z * self.wheel_track / 2.0)
        vr = linear_x - (angular_z * self.wheel_track / 2.0)

        # 2. 这里的系数可自行与 max_speed 做等比转换
        # 假设发出的期望就是基于 max_speed 的标量比例进行设置
        speed_left = vl * self.max_speed
        speed_right = vr * self.max_speed

        # 3. 幅幅保护截断 (-100 ~ 100)
        speed_left = max(min(speed_left, 100.0), -100.0)
        speed_right = max(min(speed_right, 100.0), -100.0)

        # 4. 适配硬件实际：左轮需乘以 -1 取反 (参考前面 test.py 的结论)
        speed_left_real = float(speed_left * (-1))
        speed_right_real = float(speed_right)

        # 5. 调用协议发送给底层
        self.board.set_motor_speed([[1, speed_right_real], [2, speed_left_real]])

    def watchdog_callback(self):
        now = time.time()
        # 查过设定时间没收到命令则停车
        if not self.is_stopped and (now - self.last_cmd_time > self.watchdog_timeout):
            self.get_logger().warn("Watchdog timeout! No /cmd_vel received. Force stopping.")
            self.board.set_motor_speed([[1, 0.0], [2, 0.0]])
            self.is_stopped = True

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DiffDriveTestNode('diff_drive_test_node')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 彻底关停 node 前保证强制停车
        if 'node' in locals() and hasattr(node, 'board'):
            node.board.set_motor_speed([[1, 0.0], [2, 0.0]])
            node.get_logger().info("Robot stopped.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
