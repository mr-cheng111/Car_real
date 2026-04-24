import os
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_name = 'robot_description'
    urdf_name = 'robot_gazebo.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Prefer gz pipeline when ros_gz packages are available.
    # The current robot model uses gz plugins for lidar/diffdrive.
    if shutil.which('gz'):
        try:
            get_package_share_directory('ros_gz_sim')
            get_package_share_directory('ros_gz_bridge')
            sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
            return LaunchDescription([
                DeclareLaunchArgument(
                    'use_sim_time',
                    default_value='true',
                    description='Use simulation time',
                ),
                LogInfo(msg='Detected ros_gz stack, launching robot_description/sim.launch.py'),
                sim_launch,
            ])
        except PackageNotFoundError:
            pass

    # Fallback: Gazebo Classic pipeline
    with open(urdf_model_path, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    gazebo_world_path = os.path.join(pkg_share, 'world/sim.world')

    if shutil.which('gazebo'):
        gazebo_cmd = [
            'gazebo',
            '--verbose',
            gazebo_world_path,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
        ]
    elif shutil.which('gz'):
        gazebo_cmd = ['gz', 'sim', gazebo_world_path]
    else:
        gazebo_cmd = [
            'gazebo',
            '--verbose',
            gazebo_world_path,
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn robot into Gazebo Classic from /robot_description topic.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description', '-z', '0.03'],
        output='screen',
    )

    teleop_node_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'teleop_twist_joy', 'teleop-launch.py', 'joy_vel:=/cmd_vel'],
        output='screen',
    )

    slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_gmapping'),
                'launch',
                'slam_gmapping.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
        parameters=[
            os.path.join(pkg_share, 'config/ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        LogInfo(msg='ros_gz not found, using Gazebo Classic fallback'),
        set_use_sim_time,
        set_master_uri,
        set_gazebo_ip,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        robot_localization_node,
        teleop_node_cmd,
        slam_launch_cmd,
        rviz2_node,
    ])
