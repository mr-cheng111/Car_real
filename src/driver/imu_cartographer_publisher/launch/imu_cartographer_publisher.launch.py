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

    arguments = [
        '--topic', topic,
        '--euler-topic', euler_topic,
        '--frame-id', frame_id,
        '--i2c-bus', i2c_bus,
        '--device-addr', device_addr,
        '--sample-period', sample_period,
        '--odr-hz', odr_hz,
        ['--axis-map=', axis_map],
    ]
    if LaunchConfiguration('publish_orientation').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--publish-orientation')
    if LaunchConfiguration('publish_euler').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        arguments.append('--publish-euler')
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
