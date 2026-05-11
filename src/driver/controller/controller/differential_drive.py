#!/usr/bin/python3
# coding=utf8
# 双轮差速底盘运动学(Differential drive chassis kinematic)
import math
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class DifferentialChassis:

    # 实际量测参数：
    # 轮子直径 = 8cm = 0.08m
    # 轮宽 = 32mm = 0.032m
    # 内侧距离 = 21.2cm = 0.212m
    # 轮心接地间距 (track_width) = 内侧距离 + 两个半轮宽 = 内侧距离 + 轮宽 = 0.212 + 0.032 = 0.244m
    def __init__(self, track_width=0.244, wheel_diameter=0.08):
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert linear speed (m/s) to target Motor RPS (revolutions per second)
        """
        if self.wheel_diameter == 0:
            return 0
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_speed, angular_speed):
        data = []
        # 约定: ROS 中 angular_speed > 0 表示左转(CCW)
        # 对当前硬件映射(ID1=右轮正向前进, ID2=左轮正向后退)，
        # 需要使用如下符号组合，保证左转命令不会被执行成右转。
        vl = linear_speed + angular_speed * self.track_width / 2.0
        vr = linear_speed - angular_speed * self.track_width / 2.0

        # 当前硬件映射(与 diff_drive_test_node 保持一致):
        # ID 1 -> 右轮 (正号为前进)
        # ID 2 -> 左轮 (正号为后退)
        v_s = [self.speed_covert(v) for v in [vr, -vl]]
        
        # 调试放大倍数：由于底层是以 PWM 占空比(0~100)或者 RPM 运行，而我们算出的是 0.5 这样的 RPS。
        # 此处乘以一个增益系数(例如 30.0)。这会在没编码器闭环的非标底盘上完成 RPS -> 驱动器范围的投射。
        PWM_GAIN = 30.0

        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            # 将极小的 RPS(如 0.47) 放大到足以驱动轮胎的控制值(如 14.1)
            msg.rps = float(v_s[i]) * PWM_GAIN
            data.append(msg)
            
        motors_msg = MotorsState()
        motors_msg.data = data
        return motors_msg
