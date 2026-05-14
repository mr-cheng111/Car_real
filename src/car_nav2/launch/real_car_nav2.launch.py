import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_nav2_share = get_package_share_directory('car_nav2')
    bringup_share = get_package_share_directory('robot_bringup')
    controller_share = get_package_share_directory('controller')
    imu_share = get_package_share_directory('imu_cartographer_publisher')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    robot_description_share = get_package_share_directory('robot_description')
    rplidar_share = get_package_share_directory('rplidar_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_hardware = LaunchConfiguration('start_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rf2o_in_ekf = LaunchConfiguration('use_rf2o_in_ekf')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    chassis_serial_port = LaunchConfiguration('chassis_serial_port')
    chassis_baudrate = LaunchConfiguration('chassis_baudrate')
    wheel_track = LaunchConfiguration('wheel_track')
    wheel_diameter = LaunchConfiguration('wheel_diameter')
    motor_speed_topic = LaunchConfiguration('motor_speed_topic')
    motor_speed_unit = LaunchConfiguration('motor_speed_unit')
    motor_speed_scale = LaunchConfiguration('motor_speed_scale')
    enable_speed_closed_loop = LaunchConfiguration('enable_speed_closed_loop')
    speed_kp = LaunchConfiguration('speed_kp')
    speed_ki = LaunchConfiguration('speed_ki')
    speed_kd = LaunchConfiguration('speed_kd')
    nav2_params = LaunchConfiguration('params_file')
    cartographer_state = LaunchConfiguration('cartographer_state')
    map_resolution = LaunchConfiguration('map_resolution')
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
            'euler_topic': LaunchConfiguration('imu_euler_topic'),
            'frame_id': imu_frame,
            'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
            'device_addr': LaunchConfiguration('imu_device_addr'),
            'sample_period': LaunchConfiguration('imu_sample_period'),
            'odr_hz': LaunchConfiguration('imu_odr_hz'),
            'wait_data_ready': LaunchConfiguration('imu_wait_data_ready'),
            'reject_min_accel_norm': LaunchConfiguration('imu_reject_min_accel_norm'),
            'reject_max_accel_norm': LaunchConfiguration('imu_reject_max_accel_norm'),
            'reject_max_gyro_rad_s': LaunchConfiguration('imu_reject_max_gyro_rad_s'),
            'axis_map': LaunchConfiguration('imu_axis_map'),
            'publish_orientation': LaunchConfiguration('imu_publish_orientation'),
            'publish_euler': LaunchConfiguration('imu_publish_euler'),
            'euler_from_accel': LaunchConfiguration('imu_euler_from_accel'),
            'euler_yaw_zero': LaunchConfiguration('imu_euler_yaw_zero'),
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
            'wheel_track': wheel_track,
            'wheel_diameter': wheel_diameter,
            'motor_speed_topic': motor_speed_topic,
            'motor_speed_unit': motor_speed_unit,
            'motor_speed_scale': motor_speed_scale,
            'left_feedback_sign': 1.0,
            'right_feedback_sign': -1.0,
            'enable_speed_closed_loop': enable_speed_closed_loop,
            'speed_kp': speed_kp,
            'speed_ki': speed_ki,
            'speed_kd': speed_kd,
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
                'use_wheel_speed_feedback': True,
                'motor_speed_topic': motor_speed_topic,
                'motor_speed_unit': motor_speed_unit,
                'wheel_track': wheel_track,
                'wheel_diameter': wheel_diameter,
            },
        ],
    )

    wheel_joint_state_publisher_node = Node(
        package='controller',
        executable='wheel_joint_state_publisher',
        name='wheel_joint_state_publisher',
        output='screen',
        parameters=[{
            'motor_speed_topic': motor_speed_topic,
            'motor_speed_unit': motor_speed_unit,
            'left_wheel_joint_direction': 1.0,
            'right_wheel_joint_direction': 1.0,
            'wheel_radius': 0.0175,
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
            {'publish_tf': True},
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
            {'publish_tf': True},
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=IfCondition(use_rf2o_in_ekf),
    )

    hardware_bringup = GroupAction(
        actions=[
            robot_description_launch,
            c1_lidar_launch,
            external_imu_launch,
            robot_controller_node,
            odom_publisher_node,
            wheel_joint_state_publisher_node,
            rf2o_laser_odometry_node,
            ekf_filter_node_without_rf2o,
            ekf_filter_node_with_rf2o,
        ],
        condition=IfCondition(start_hardware),
    )

    cartographer_localization_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-configuration_directory', os.path.join(bringup_share, 'config'),
                    '-configuration_basename', 'cartographer_2d_real_localization.lua',
                    '-load_state_filename', cartographer_state,
                ],
                remappings=[
                    ('scan', '/scan'),
                    ('odom', '/odom'),
                    ('imu', '/imu'),
                ],
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-resolution', map_resolution,
                    '-publish_period_sec', '0.5',
                ],
            ),
        ],
    )

    nav2_bringup = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params,
                }.items(),
            )
        ],
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rf2o_in_ekf', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyS8'),
        DeclareLaunchArgument('chassis_serial_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('chassis_baudrate', default_value='115200'),
        DeclareLaunchArgument('wheel_track', default_value='0.2948'),
        DeclareLaunchArgument('wheel_diameter', default_value='0.035'),
        DeclareLaunchArgument('motor_speed_topic', default_value='/motor_speed'),
        DeclareLaunchArgument('motor_speed_unit', default_value='rpm'),
        DeclareLaunchArgument('motor_speed_scale', default_value='1.0'),
        DeclareLaunchArgument('enable_speed_closed_loop', default_value='true'),
        DeclareLaunchArgument('speed_kp', default_value='6.0'),
        DeclareLaunchArgument('speed_ki', default_value='1.0'),
        DeclareLaunchArgument('speed_kd', default_value='0.0'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_link'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('imu_i2c_bus', default_value='4'),
        DeclareLaunchArgument('imu_device_addr', default_value='0x6A'),
        DeclareLaunchArgument('imu_sample_period', default_value='0.005'),
        DeclareLaunchArgument('imu_odr_hz', default_value='208'),
        DeclareLaunchArgument('imu_wait_data_ready', default_value='true'),
        DeclareLaunchArgument('imu_reject_min_accel_norm', default_value='6.0'),
        DeclareLaunchArgument('imu_reject_max_accel_norm', default_value='13.0'),
        DeclareLaunchArgument('imu_reject_max_gyro_rad_s', default_value='8.0'),
        DeclareLaunchArgument('imu_axis_map', default_value='-y,-x,-z'),
        DeclareLaunchArgument('imu_publish_orientation', default_value='true'),
        DeclareLaunchArgument('imu_publish_euler', default_value='true'),
        DeclareLaunchArgument('imu_euler_topic', default_value='/imu/euler_deg'),
        DeclareLaunchArgument('imu_euler_from_accel', default_value='true'),
        DeclareLaunchArgument('imu_euler_yaw_zero', default_value='false'),
        DeclareLaunchArgument('imu_print_debug', default_value='true'),
        DeclareLaunchArgument(
            'cartographer_state',
            default_value='src/car_nav2/maps/cartographer/latest.pbstream',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(car_nav2_share, 'param', 'car_nav2.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_share, 'config', 'default.rviz'),
        ),
        hardware_bringup,
        cartographer_localization_node,
        nav2_bringup,
        rviz_node,
    ])
