from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_teleop', default_value='true'),
        DeclareLaunchArgument('use_rf2o_in_ekf', default_value='true'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyS8'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'real_robot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_teleop': LaunchConfiguration('enable_teleop'),
                'use_rf2o_in_ekf': LaunchConfiguration('use_rf2o_in_ekf'),
                'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
                'map_resolution': LaunchConfiguration('map_resolution'),
            }.items(),
        ),
    ])
