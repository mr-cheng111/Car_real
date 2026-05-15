import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_rviz_config = '/home/mr-cheng/Car_real/src/robot_description/rviz/default.rviz'
    exploration_share = get_package_share_directory('exploration')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    robot_bringup_share = get_package_share_directory('robot_bringup')
    car_nav2_share = get_package_share_directory('car_nav2')
    robot_description_share = get_package_share_directory('robot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    explore_params = LaunchConfiguration('explore_params')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_share, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel',
        }.items(),
    )

    nav2_navigation = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                    'autostart': 'true',
                }.items(),
            )
        ],
    )

    frontier_explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='exploration',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[
                    explore_params,
                    {
                        'nav_action_name': 'navigate_to_pose',
                        'cmd_vel_topic': '/cmd_vel',
                        'enable_return_home': True,
                        'num_random_goals': 0,
                    },
                ],
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', source_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(car_nav2_share, 'param', 'car_nav2.yaml'),
        ),
        DeclareLaunchArgument(
            'explore_params',
            default_value=os.path.join(exploration_share, 'config', 'explore_params.yaml'),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_bringup,
        nav2_navigation,
        frontier_explorer,
        rviz,
    ])
