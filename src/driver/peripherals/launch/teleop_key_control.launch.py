from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic used by keyboard teleop'
    )

    teleop_key_control_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    ld = LaunchDescription()
    ld.add_action(cmd_vel_topic_arg)
    ld.add_action(teleop_key_control_node)

    return ld

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
