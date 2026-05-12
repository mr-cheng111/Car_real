from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    baudrate = LaunchConfiguration('baudrate')
    serial_timeout = LaunchConfiguration('serial_timeout')
    publish_rate = LaunchConfiguration('publish_rate')
    raw_topic = LaunchConfiguration('raw_topic')
    motors_topic = LaunchConfiguration('motors_topic')
    left_motor_id = LaunchConfiguration('left_motor_id')
    right_motor_id = LaunchConfiguration('right_motor_id')
    left_speed_offset = LaunchConfiguration('left_speed_offset')
    right_speed_offset = LaunchConfiguration('right_speed_offset')
    speed_scale = LaunchConfiguration('speed_scale')
    left_speed_sign = LaunchConfiguration('left_speed_sign')
    right_speed_sign = LaunchConfiguration('right_speed_sign')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('serial_timeout', default_value='0.02'),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('raw_topic', default_value='/motor_speed/raw_array'),
        DeclareLaunchArgument('motors_topic', default_value='/motor_speed'),
        DeclareLaunchArgument('left_motor_id', default_value='2'),
        DeclareLaunchArgument('right_motor_id', default_value='1'),
        DeclareLaunchArgument('left_speed_offset', default_value='0'),
        DeclareLaunchArgument('right_speed_offset', default_value='4'),
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
        DeclareLaunchArgument('left_speed_sign', default_value='-1.0'),
        DeclareLaunchArgument('right_speed_sign', default_value='1.0'),
        Node(
            package='motor_speed_reader',
            executable='motor_speed_reader',
            name='motor_speed_reader',
            output='screen',
            parameters=[{
                'device': device,
                'baudrate': baudrate,
                'serial_timeout': serial_timeout,
                'publish_rate': publish_rate,
                'raw_topic': raw_topic,
                'motors_topic': motors_topic,
                'left_motor_id': left_motor_id,
                'right_motor_id': right_motor_id,
                'left_speed_offset': left_speed_offset,
                'right_speed_offset': right_speed_offset,
                'speed_scale': speed_scale,
                'left_speed_sign': left_speed_sign,
                'right_speed_sign': right_speed_sign,
            }],
        ),
    ])
