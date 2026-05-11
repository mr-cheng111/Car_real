import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_name = 'robot_description'
    pkg_share = get_package_share_directory(package_name)
    bringup_share = get_package_share_directory('robot_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_gazebo.urdf')
    world_file = os.path.join(pkg_share, 'world', 'sim.world')
    ekf_config = os.path.join(bringup_share, 'config', 'ekf.yaml')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    nav2_launch_path = os.path.join(
        get_package_share_directory('car_nav2'),
        'launch',
        'car_nav2.launch.py',
    )
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gazebo_cmd = [
        'gazebo',
        '--verbose',
        world_file,
        '-s',
        'libgazebo_ros_init.so',
        '-s',
        'libgazebo_ros_factory.so',
    ]

    set_master_uri = SetEnvironmentVariable(
        name='GAZEBO_MASTER_URI',
        value='http://127.0.0.1:11346',
    )

    set_gazebo_ip = SetEnvironmentVariable(
        name='GAZEBO_IP',
        value='127.0.0.1',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    start_gazebo_cmd = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen',
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description', '-z', '0.03'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        LogInfo(msg='Using Gazebo Classic pipeline for robot_model.launch.py'),
        set_use_sim_time,
        set_master_uri,
        set_gazebo_ip,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        robot_localization_node,
        nav2_launch_cmd,
        rviz2_node,
    ])
