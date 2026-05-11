#!/usr/bin/env python3
import argparse
import fcntl
import math
import os
import signal
import struct
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None
    Node = object
    Imu = None
    remove_ros_args = None


I2C_SLAVE_FORCE = 0x0706

CTRL1_XL = 0x10
CTRL2_G = 0x11
OUTX_L_G = 0x22

DEFAULT_GYRO_SCALE_RAD = 0.000152716
DEFAULT_ACCEL_SCALE_G = 0.061 / 1000.0
GRAVITY = 9.80665


class MahonyAHRS:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.e_int = [0.0, 0.0, 0.0]

    def update(self, gx, gy, gz, ax, ay, az, dt):
        if dt <= 0.0:
            return

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-9:
            return

        ax /= norm
        ay /= norm
        az /= norm

        q0, q1, q2, q3 = self.q

        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        ex = ay * vz - az * vy
        ey = az * vx - ax * vz
        ez = ax * vy - ay * vx

        if self.ki > 0.0:
            self.e_int[0] += ex * self.ki * dt
            self.e_int[1] += ey * self.ki * dt
            self.e_int[2] += ez * self.ki * dt
        else:
            self.e_int = [0.0, 0.0, 0.0]

        gx += self.kp * ex + self.e_int[0]
        gy += self.kp * ey + self.e_int[1]
        gz += self.kp * ez + self.e_int[2]

        half_dt = 0.5 * dt
        q0_old, q1_old, q2_old, q3_old = q0, q1, q2, q3

        q0 += (-q1_old * gx - q2_old * gy - q3_old * gz) * half_dt
        q1 += (q0_old * gx + q2_old * gz - q3_old * gy) * half_dt
        q2 += (q0_old * gy - q1_old * gz + q3_old * gx) * half_dt
        q3 += (q0_old * gz + q1_old * gy - q2_old * gx) * half_dt

        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm > 1e-9:
            self.q = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

    def quaternion_xyzw(self):
        q0, q1, q2, q3 = self.q
        return q1, q2, q3, q0

    def euler_deg(self):
        q0, q1, q2, q3 = self.q
        roll = math.atan2(
            2.0 * (q2 * q3 + q0 * q1),
            1.0 - 2.0 * (q1 * q1 + q2 * q2),
        )
        pitch_val = -2.0 * q1 * q3 + 2.0 * q0 * q2
        pitch_val = max(-1.0, min(1.0, pitch_val))
        pitch = math.asin(pitch_val)
        yaw = math.atan2(
            2.0 * (q1 * q2 + q0 * q3),
            1.0 - 2.0 * (q2 * q2 + q3 * q3),
        )
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class I2CImu:
    def __init__(self, bus, addr, gyro_scale_rad, accel_scale_g):
        self.path = f"/dev/i2c-{bus}"
        self.addr = addr
        self.gyro_scale_rad = gyro_scale_rad
        self.accel_scale_ms2 = accel_scale_g * GRAVITY
        self.fd = None

    def open(self):
        self.fd = os.open(self.path, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE_FORCE, self.addr)
        os.write(self.fd, bytes([CTRL2_G, 0x10]))
        time.sleep(0.01)
        os.write(self.fd, bytes([CTRL1_XL, 0x10]))
        time.sleep(0.01)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    @staticmethod
    def _i16(lo, hi):
        return struct.unpack("<h", bytes([lo, hi]))[0]

    def read_sensor_units(self):
        os.write(self.fd, bytes([OUTX_L_G]))
        data = os.read(self.fd, 12)
        if len(data) != 12:
            raise RuntimeError(f"IMU read length unexpected: {len(data)}")

        gx = self._i16(data[0], data[1]) * self.gyro_scale_rad
        gy = self._i16(data[2], data[3]) * self.gyro_scale_rad
        gz = self._i16(data[4], data[5]) * self.gyro_scale_rad

        ax = self._i16(data[6], data[7]) * self.accel_scale_ms2
        ay = self._i16(data[8], data[9]) * self.accel_scale_ms2
        az = self._i16(data[10], data[11]) * self.accel_scale_ms2
        return (gx, gy, gz), (ax, ay, az)


def parse_axis_map(axis_map):
    axes = {"x": 0, "y": 1, "z": 2}
    parts = [part.strip().lower() for part in axis_map.split(",")]
    if len(parts) != 3:
        raise ValueError("axis map must have 3 comma-separated entries, e.g. y,-x,z")

    result = []
    used = set()
    for part in parts:
        sign = 1.0
        if part.startswith("-"):
            sign = -1.0
            part = part[1:]
        elif part.startswith("+"):
            part = part[1:]

        if part not in axes:
            raise ValueError(f"bad axis '{part}' in axis map")
        if part in used:
            raise ValueError("axis map cannot reuse a sensor axis")
        used.add(part)
        result.append((axes[part], sign))
    return result


def map_vec(vec, mapping):
    return tuple(sign * vec[index] for index, sign in mapping)


def clamp(value, limit):
    return max(-limit, min(limit, value))


def calibrate_gyro(imu, samples, sample_period, wait_sec, max_abs_rad_s):
    print("开始陀螺仪静态校正，请保持机器人完全静止")
    time.sleep(wait_sec)

    sums = [0.0, 0.0, 0.0]
    valid = 0
    for _ in range(samples):
        try:
            gyro, _ = imu.read_sensor_units()
            if max(abs(gyro[0]), abs(gyro[1]), abs(gyro[2])) <= max_abs_rad_s:
                sums[0] += gyro[0]
                sums[1] += gyro[1]
                sums[2] += gyro[2]
                valid += 1
        except Exception as exc:
            print(f"校正采样失败: {exc}", file=sys.stderr)
        time.sleep(sample_period)

    if valid == 0:
        print("警告：没有有效校正样本，gyro bias 使用 0", file=sys.stderr)
        return (0.0, 0.0, 0.0), valid

    bias = (sums[0] / valid, sums[1] / valid, sums[2] / valid)
    print(
        "gyro_bias(rad/s): "
        f"gx={bias[0]:.6f}, gy={bias[1]:.6f}, gz={bias[2]:.6f}, "
        f"valid={valid}/{samples}"
    )
    return bias, valid


class ImuCartographerNode(Node):
    def __init__(self, args):
        super().__init__("imu_cartographer_publisher")
        self.args = args
        self.mapping = parse_axis_map(args.axis_map)
        self.imu = I2CImu(args.i2c_bus, args.device_addr, args.gyro_scale_rad, args.accel_scale_g)
        self.imu.open()

        self.bias, _ = calibrate_gyro(
            self.imu,
            args.calibration_samples,
            args.sample_period,
            args.calibration_wait_sec,
            args.calibration_max_gyro_rad_s,
        )

        self.ahrs = MahonyAHRS(args.mahony_kp, args.mahony_ki)
        self.pub = self.create_publisher(Imu, args.topic, 20)
        self.last_time = time.monotonic()
        self.filtered_gyro = [0.0, 0.0, 0.0]
        self.filtered_accel = [0.0, 0.0, 0.0]
        self.timer = self.create_timer(args.sample_period, self.publish_once)

        self.get_logger().info(
            f"publishing Cartographer IMU: topic={args.topic}, frame={args.frame_id}, "
            f"axis_map={args.axis_map}, i2c={self.imu.path}, addr=0x{args.device_addr:02X}"
        )

    def publish_once(self):
        try:
            gyro_sensor, accel_sensor = self.imu.read_sensor_units()
            gyro_sensor = tuple(gyro_sensor[i] - self.bias[i] for i in range(3))

            gyro_robot = map_vec(gyro_sensor, self.mapping)
            accel_robot = map_vec(accel_sensor, self.mapping)

            gyro_robot = tuple(clamp(v, self.args.max_gyro_rad_s) for v in gyro_robot)
            accel_robot = tuple(clamp(v, self.args.max_accel_m_s2) for v in accel_robot)

            alpha = self.args.low_pass_alpha
            for i in range(3):
                self.filtered_gyro[i] = alpha * gyro_robot[i] + (1.0 - alpha) * self.filtered_gyro[i]
                self.filtered_accel[i] = alpha * accel_robot[i] + (1.0 - alpha) * self.filtered_accel[i]

            now = time.monotonic()
            dt = now - self.last_time
            self.last_time = now
            self.ahrs.update(
                self.filtered_gyro[0],
                self.filtered_gyro[1],
                self.filtered_gyro[2],
                self.filtered_accel[0],
                self.filtered_accel[1],
                self.filtered_accel[2],
                dt,
            )

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.args.frame_id

            qx, qy, qz, qw = self.ahrs.quaternion_xyzw()
            msg.orientation.x = qx if self.args.publish_orientation else 0.0
            msg.orientation.y = qy if self.args.publish_orientation else 0.0
            msg.orientation.z = qz if self.args.publish_orientation else 0.0
            msg.orientation.w = qw if self.args.publish_orientation else 1.0
            msg.orientation_covariance = (
                [self.args.orientation_covariance, 0.0, 0.0,
                 0.0, self.args.orientation_covariance, 0.0,
                 0.0, 0.0, self.args.orientation_covariance]
                if self.args.publish_orientation
                else [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )

            msg.angular_velocity.x = self.filtered_gyro[0]
            msg.angular_velocity.y = self.filtered_gyro[1]
            msg.angular_velocity.z = self.filtered_gyro[2]
            msg.angular_velocity_covariance = [
                self.args.angular_velocity_covariance, 0.0, 0.0,
                0.0, self.args.angular_velocity_covariance, 0.0,
                0.0, 0.0, self.args.angular_velocity_covariance,
            ]

            msg.linear_acceleration.x = self.filtered_accel[0]
            msg.linear_acceleration.y = self.filtered_accel[1]
            msg.linear_acceleration.z = self.filtered_accel[2]
            msg.linear_acceleration_covariance = [
                self.args.linear_acceleration_covariance, 0.0, 0.0,
                0.0, self.args.linear_acceleration_covariance, 0.0,
                0.0, 0.0, self.args.linear_acceleration_covariance,
            ]

            self.pub.publish(msg)

            if self.args.print_debug:
                roll, pitch, yaw = self.ahrs.euler_deg()
                accel_norm = math.sqrt(sum(v * v for v in self.filtered_accel))
                print(
                    f"roll={roll:8.2f} pitch={pitch:8.2f} yaw={yaw:8.2f} | "
                    f"gyro=({self.filtered_gyro[0]: .4f}, {self.filtered_gyro[1]: .4f}, {self.filtered_gyro[2]: .4f}) rad/s | "
                    f"accel=({self.filtered_accel[0]: .3f}, {self.filtered_accel[1]: .3f}, {self.filtered_accel[2]: .3f}) "
                    f"| |a|={accel_norm:.3f}",
                    end="\r",
                    flush=True,
                )
        except Exception as exc:
            self.get_logger().warn(f"IMU publish failed: {exc}")

    def destroy_node(self):
        try:
            self.imu.close()
        finally:
            super().destroy_node()


def print_only_loop(args):
    mapping = parse_axis_map(args.axis_map)
    imu = I2CImu(args.i2c_bus, args.device_addr, args.gyro_scale_rad, args.accel_scale_g)
    imu.open()
    try:
        bias, _ = calibrate_gyro(
            imu,
            args.calibration_samples,
            args.sample_period,
            args.calibration_wait_sec,
            args.calibration_max_gyro_rad_s,
        )

        ahrs = MahonyAHRS(args.mahony_kp, args.mahony_ki)
        filtered_gyro = [0.0, 0.0, 0.0]
        filtered_accel = [0.0, 0.0, 0.0]
        last_time = time.monotonic()

        print("开始输出机器人坐标系下的 IMU 数据，Ctrl-C 停止")
        print("默认映射: robot_x=-sensor_y, robot_y=-sensor_x, robot_z=-sensor_z")
        while True:
            gyro_sensor, accel_sensor = imu.read_sensor_units()
            gyro_sensor = tuple(gyro_sensor[i] - bias[i] for i in range(3))
            gyro_robot = map_vec(gyro_sensor, mapping)
            accel_robot = map_vec(accel_sensor, mapping)

            alpha = args.low_pass_alpha
            for i in range(3):
                filtered_gyro[i] = alpha * gyro_robot[i] + (1.0 - alpha) * filtered_gyro[i]
                filtered_accel[i] = alpha * accel_robot[i] + (1.0 - alpha) * filtered_accel[i]

            now = time.monotonic()
            dt = now - last_time
            last_time = now
            ahrs.update(
                filtered_gyro[0],
                filtered_gyro[1],
                filtered_gyro[2],
                filtered_accel[0],
                filtered_accel[1],
                filtered_accel[2],
                dt,
            )
            roll, pitch, yaw = ahrs.euler_deg()
            accel_norm = math.sqrt(sum(v * v for v in filtered_accel))
            print(
                f"roll={roll:8.2f} pitch={pitch:8.2f} yaw={yaw:8.2f} | "
                f"gyro=({filtered_gyro[0]: .4f}, {filtered_gyro[1]: .4f}, {filtered_gyro[2]: .4f}) rad/s | "
                f"accel=({filtered_accel[0]: .3f}, {filtered_accel[1]: .3f}, {filtered_accel[2]: .3f}) m/s^2 | "
                f"|a|={accel_norm:.3f}",
                end="\r",
                flush=True,
            )
            time.sleep(args.sample_period)
    finally:
        imu.close()


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Read ASM330LHH over I2C and publish Cartographer-compatible sensor_msgs/Imu."
    )
    parser.add_argument("--topic", default="/imu", help="ROS topic for sensor_msgs/Imu")
    parser.add_argument("--frame-id", default="imu_link", help="IMU frame_id; match Cartographer tracking_frame")
    parser.add_argument("--i2c-bus", type=int, default=4)
    parser.add_argument("--device-addr", type=lambda value: int(value, 0), default=0x6A)
    parser.add_argument("--sample-period", type=float, default=0.08)
    parser.add_argument("--calibration-samples", type=int, default=50)
    parser.add_argument("--calibration-wait-sec", type=float, default=1.0)
    parser.add_argument("--calibration-max-gyro-rad-s", type=float, default=0.35)
    parser.add_argument("--gyro-scale-rad", type=float, default=DEFAULT_GYRO_SCALE_RAD)
    parser.add_argument("--accel-scale-g", type=float, default=DEFAULT_ACCEL_SCALE_G)
    parser.add_argument("--low-pass-alpha", type=float, default=0.35)
    parser.add_argument("--max-gyro-rad-s", type=float, default=4.0)
    parser.add_argument("--max-accel-m-s2", type=float, default=30.0)
    parser.add_argument(
        "--axis-map",
        default="-y,-x,-z",
        help=(
            "robot axes expressed as sensor axes. Default -y,-x,-z means "
            "robot_x=-sensor_y, robot_y=-sensor_x, robot_z=-sensor_z."
        ),
    )
    parser.add_argument("--mahony-kp", type=float, default=4.2)
    parser.add_argument("--mahony-ki", type=float, default=0.0)
    parser.add_argument("--angular-velocity-covariance", type=float, default=0.02)
    parser.add_argument("--linear-acceleration-covariance", type=float, default=0.08)
    parser.add_argument("--orientation-covariance", type=float, default=0.05)
    parser.add_argument(
        "--publish-orientation",
        action="store_true",
        help="Publish Mahony orientation. Leave disabled for Cartographer to avoid feeding drifting yaw.",
    )
    parser.add_argument("--print-debug", action="store_true")
    parser.add_argument("--print-only", action="store_true", help="Do not use ROS; only print mapped values.")
    return parser


def main():
    argv = sys.argv[1:]
    if remove_ros_args is not None:
        argv = remove_ros_args(args=argv)
    args = build_arg_parser().parse_args(argv)

    if args.print_only:
        print_only_loop(args)
        return

    if rclpy is None:
        print("rclpy/sensor_msgs not available. Source ROS 2 first, or run with --print-only.", file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = ImuCartographerNode(args)

    def handle_signal(_signum, _frame):
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
