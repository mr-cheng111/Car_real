from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
