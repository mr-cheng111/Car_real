from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    topic = LaunchConfiguration('topic')
    euler_topic = LaunchConfiguration('euler_topic')
    frame_id = LaunchConfiguration('frame_id')
    i2c_bus = LaunchConfiguration('i2c_bus')
    device_addr = LaunchConfiguration('device_addr')
    sample_period = LaunchConfiguration('sample_period')
    odr_hz = LaunchConfiguration('odr_hz')
    axis_map = LaunchConfiguration('axis_map')
    reject_min_accel_norm = LaunchConfiguration('reject_min_accel_norm')
    reject_max_accel_norm = LaunchConfiguration('reject_max_accel_norm')
    reject_max_gyro_rad_s = LaunchConfiguration('reject_max_gyro_rad_s')

    arguments = [
        '--topic', topic,
        '--euler-topic', euler_topic,
        '--frame-id', frame_id,
        '--i2c-bus', i2c_bus,
        '--device-addr', device_addr,
        '--sample-period', sample_period,
        '--odr-hz', odr_hz,
        '--reject-min-accel-norm', reject_min_accel_norm,
        '--reject-max-accel-norm', reject_max_accel_norm,
        '--reject-max-gyro-rad-s', reject_max_gyro_rad_s,
        ['--axis-map=', axis_map],
    ]
    if LaunchConfiguration('wait_data_ready').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--wait-data-ready')
    if LaunchConfiguration('publish_orientation').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--publish-orientation')
    if LaunchConfiguration('publish_euler').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--publish-euler')
    if LaunchConfiguration('euler_from_accel').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--euler-from-accel')
    if LaunchConfiguration('euler_yaw_zero').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--euler-yaw-zero')
    if LaunchConfiguration('print_debug').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--print-debug')

    node = Node(
        package='imu_cartographer_publisher',
        executable='imu_cartographer_publisher',
        name='imu_cartographer_publisher',
        output='screen',
        arguments=arguments,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/imu'),
        DeclareLaunchArgument('euler_topic', default_value='/imu/euler_deg'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('i2c_bus', default_value='4'),
        DeclareLaunchArgument('device_addr', default_value='0x6A'),
        DeclareLaunchArgument('sample_period', default_value='0.005'),
        DeclareLaunchArgument('odr_hz', default_value='208'),
        DeclareLaunchArgument('wait_data_ready', default_value='true'),
        DeclareLaunchArgument('reject_min_accel_norm', default_value='6.0'),
        DeclareLaunchArgument('reject_max_accel_norm', default_value='13.0'),
        DeclareLaunchArgument('reject_max_gyro_rad_s', default_value='8.0'),
        DeclareLaunchArgument(
            'publish_orientation',
            default_value='true',
            description='Publish Mahony AHRS orientation in sensor_msgs/Imu.',
        ),
        DeclareLaunchArgument(
            'publish_euler',
            default_value='true',
            description='Publish Mahony AHRS Euler angles in degrees as geometry_msgs/Vector3Stamped.',
        ),
        DeclareLaunchArgument(
            'euler_from_accel',
            default_value='true',
            description='Publish Euler roll/pitch from accelerometer and yaw=0 for stable low-dynamic display.',
        ),
        DeclareLaunchArgument(
            'euler_yaw_zero',
            default_value='false',
            description='Force Euler yaw output to 0 when not using euler_from_accel.',
        ),
        DeclareLaunchArgument(
            'axis_map',
            default_value='-y,-x,-z',
            description='Robot axes expressed as sensor axes: robot_x,robot_y,robot_z.',
        ),
        DeclareLaunchArgument(
            'print_debug',
            default_value='true',
            description='Enable roll/pitch/yaw and raw IMU debug output.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
