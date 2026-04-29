import os
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_name = 'robot_description'
    urdf_name = 'robot_gazebo.urdf'

    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rf2o = LaunchConfiguration('enable_rf2o')
    rf2o_publish_tf = LaunchConfiguration('rf2o_publish_tf')

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=IfCondition(enable_rf2o),
        parameters=[{
            'use_sim_time': use_sim_time,
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': rf2o_publish_tf,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'init_pose_from_imu_topic': '/imu',
            'motion_filter_linear_m': 0.06,
            'motion_filter_angular_rad': 0.0020944,
            'freq': 30.0,
        }],
    )

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
                DeclareLaunchArgument(
                    'enable_rf2o',
                    default_value='false',
                    description='Start rf2o laser odometry node and publish /odom_rf2o',
                ),
                DeclareLaunchArgument(
                    'rf2o_publish_tf',
                    default_value='false',
                    description='Whether rf2o should publish odom->base_link TF',
                ),
                LogInfo(msg='Detected ros_gz stack, launching robot_description/sim.launch.py'),
                sim_launch,
                rf2o_node,
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
        DeclareLaunchArgument(
            'enable_rf2o',
            default_value='false',
            description='Start rf2o laser odometry node and publish /odom_rf2o',
        ),
        DeclareLaunchArgument(
            'rf2o_publish_tf',
            default_value='false',
            description='Whether rf2o should publish odom->base_link TF',
        ),
        LogInfo(msg='ros_gz not found, using Gazebo Classic fallback'),
        set_use_sim_time,
        set_master_uri,
        set_gazebo_ip,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        robot_localization_node,
        rf2o_node,
        slam_launch_cmd,
        rviz2_node,
    ])
