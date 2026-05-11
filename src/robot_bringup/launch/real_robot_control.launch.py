import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rf2o_in_ekf', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyS8'),
        DeclareLaunchArgument('chassis_serial_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('chassis_baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_link'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('imu_i2c_bus', default_value='4'),
        DeclareLaunchArgument('imu_device_addr', default_value='0x6A'),
        DeclareLaunchArgument('imu_sample_period', default_value='0.08'),
        DeclareLaunchArgument('imu_axis_map', default_value='y,-x,z'),
        DeclareLaunchArgument('imu_print_debug', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'real_robot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_teleop': 'true',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'enable_auto_navigation': 'false',
                'enable_frontier_exploration': 'false',
                'use_rf2o_in_ekf': LaunchConfiguration('use_rf2o_in_ekf'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
                'chassis_serial_port': LaunchConfiguration('chassis_serial_port'),
                'chassis_baudrate': LaunchConfiguration('chassis_baudrate'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'map_resolution': LaunchConfiguration('map_resolution'),
                'imu_i2c_bus': LaunchConfiguration('imu_i2c_bus'),
                'imu_device_addr': LaunchConfiguration('imu_device_addr'),
                'imu_sample_period': LaunchConfiguration('imu_sample_period'),
                'imu_axis_map': LaunchConfiguration('imu_axis_map'),
                'imu_print_debug': LaunchConfiguration('imu_print_debug'),
            }.items(),
        ),
    ])
