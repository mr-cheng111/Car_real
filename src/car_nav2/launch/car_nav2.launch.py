import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    car_nav2_dir = get_package_share_directory('car_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_description_dir = get_package_share_directory('robot_description')

    urdf_path = os.path.join(robot_description_dir, 'urdf', 'robot_gazebo.urdf')
    world_path = os.path.join(robot_description_dir, 'world', 'sim.world')
    ekf_path = os.path.join(robot_description_dir, 'config', 'ekf.yaml')
    rviz_config_path = os.path.join(robot_description_dir, 'config', 'default.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_sim = LaunchConfiguration('start_sim')
    map_yaml_path = LaunchConfiguration('map')
    nav2_param_path = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    declare_start_sim = DeclareLaunchArgument(
        'start_sim',
        default_value='true',
        description='Start Gazebo and spawn robot before Nav2',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(car_nav2_dir, 'maps', 'Sim_Map', 'sim_map.yaml'),
        description='Full path to map yaml file',
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(car_nav2_dir, 'param', 'car_nav2.yaml'),
        description='Full path to Nav2 params file',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 with default config',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    set_master_uri = SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11346')
    set_gazebo_ip = SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1')
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
        ],
        output='screen',
        condition=IfCondition(start_sim),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(start_sim),
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description', '-z', '0.03'],
        output='screen',
    )
    delayed_spawn_entity = TimerAction(
        period=3.0,
        actions=[spawn_entity_node],
        condition=IfCondition(start_sim),
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
        parameters=[ekf_path, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_sim),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
        }.items(),
        condition=UnlessCondition(start_sim),
    )

    delayed_nav2_bringup_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
                launch_arguments={
                    'map': map_yaml_path,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_param_path,
                }.items(),
            )
        ],
        condition=IfCondition(start_sim),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_start_sim,
        declare_map,
        declare_params,
        declare_use_rviz,
        set_use_sim_time,
        set_master_uri,
        set_gazebo_ip,
        gazebo_cmd,
        robot_state_publisher_node,
        delayed_spawn_entity,
        robot_localization_node,
        nav2_bringup_launch,
        delayed_nav2_bringup_launch,
        rviz_node,
    ])
