import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    car_nav2_share = get_package_share_directory('car_nav2')
    controller_share = get_package_share_directory('controller')
    exploration_share = get_package_share_directory('exploration')
    imu_share = get_package_share_directory('imu_cartographer_publisher')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    robot_description_share = get_package_share_directory('robot_description')
    rplidar_share = get_package_share_directory('rplidar_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_teleop = LaunchConfiguration('enable_teleop')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_auto_navigation = LaunchConfiguration('enable_auto_navigation')
    enable_frontier_exploration = LaunchConfiguration('enable_frontier_exploration')
    use_rf2o_in_ekf = LaunchConfiguration('use_rf2o_in_ekf')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    chassis_serial_port = LaunchConfiguration('chassis_serial_port')
    chassis_baudrate = LaunchConfiguration('chassis_baudrate')
    nav2_params = LaunchConfiguration('nav2_params')
    explore_params = LaunchConfiguration('explore_params')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    c1_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_share, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'channel_type': 'serial',
            'serial_port': lidar_serial_port,
            'serial_baudrate': '460800',
            'frame_id': lidar_frame,
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
        }.items(),
    )

    external_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_share, 'launch', 'imu_cartographer_publisher.launch.py')
        ),
        launch_arguments={
            'topic': '/imu',
            'frame_id': imu_frame,
            'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
            'device_addr': LaunchConfiguration('imu_device_addr'),
            'sample_period': LaunchConfiguration('imu_sample_period'),
            'axis_map': LaunchConfiguration('imu_axis_map'),
            'print_debug': LaunchConfiguration('imu_print_debug'),
        }.items(),
    )

    robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
        parameters=[{
            'device': chassis_serial_port,
            'baudrate': chassis_baudrate,
            'imu_frame': imu_frame,
            'publish_imu': False,
            'cmd_vel_topic': cmd_vel_topic,
        }],
    )

    odom_publisher_node = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[
            os.path.join(controller_share, 'config', 'calibrate_params.yaml'),
            {
                'base_frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'pub_odom_topic': True,
            },
        ],
    )

    wheel_joint_state_publisher_node = Node(
        package='controller',
        executable='wheel_joint_state_publisher',
        name='wheel_joint_state_publisher',
        output='screen',
        parameters=[{
            'cmd_vel_topic': cmd_vel_topic,
            'wheel_radius': 0.05,
            'wheel_track': 0.2948,
        }],
    )

    rf2o_laser_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'init_pose_from_topic': '',
            'init_pose_from_imu_topic': '/imu',
            'motion_filter_linear_m': 0.01,
            'motion_filter_angular_rad': 0.01,
            'freq': 12.0,
        }],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    ekf_filter_node_without_rf2o = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'ekf_external_imu.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=UnlessCondition(use_rf2o_in_ekf),
    )

    ekf_filter_node_with_rf2o = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'ekf_external_imu_rf2o.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=IfCondition(use_rf2o_in_ekf),
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(bringup_share, 'config'),
            '-configuration_basename', 'cartographer_2d_real.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', LaunchConfiguration('map_resolution'),
            '-publish_period_sec', '0.5',
        ],
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
        condition=IfCondition(enable_auto_navigation),
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
                        'cmd_vel_topic': cmd_vel_topic,
                        'enable_return_home': True,
                        'num_random_goals': 0,
                    },
                ],
            )
        ],
        condition=IfCondition(enable_frontier_exploration),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    teleop_key_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', cmd_vel_topic)],
        condition=IfCondition(enable_teleop),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_teleop', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('enable_auto_navigation', default_value='true'),
        DeclareLaunchArgument('enable_frontier_exploration', default_value='true'),
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
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(car_nav2_share, 'param', 'car_nav2.yaml'),
        ),
        DeclareLaunchArgument(
            'explore_params',
            default_value=os.path.join(exploration_share, 'config', 'explore_params.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_share, 'config', 'default.rviz'),
        ),
        DeclareLaunchArgument('imu_i2c_bus', default_value='4'),
        DeclareLaunchArgument('imu_device_addr', default_value='0x6A'),
        DeclareLaunchArgument('imu_sample_period', default_value='0.08'),
        DeclareLaunchArgument('imu_axis_map', default_value='x,y,z'),
        DeclareLaunchArgument('imu_print_debug', default_value='true'),
        robot_description_launch,
        c1_lidar_launch,
        external_imu_launch,
        robot_controller_node,
        odom_publisher_node,
        wheel_joint_state_publisher_node,
        rf2o_laser_odometry_node,
        ekf_filter_node_without_rf2o,
        ekf_filter_node_with_rf2o,
        cartographer_node,
        occupancy_grid_node,
        nav2_navigation,
        frontier_explorer,
        rviz_node,
        teleop_key_node,
    ])
