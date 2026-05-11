import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_share = get_package_share_directory("imu_cartographer_publisher")

    topic = LaunchConfiguration("topic")
    frame_id = LaunchConfiguration("frame_id")
    i2c_bus = LaunchConfiguration("i2c_bus")
    device_addr = LaunchConfiguration("device_addr")
    sample_period = LaunchConfiguration("sample_period")
    axis_map = LaunchConfiguration("axis_map")

    imu_arguments = [
        "--topic", topic,
        "--frame-id", frame_id,
        "--i2c-bus", i2c_bus,
        "--device-addr", device_addr,
        "--sample-period", sample_period,
        ["--axis-map=", axis_map],
        "--publish-orientation",
    ]
    if LaunchConfiguration("print_debug").perform(context).lower() in ("1", "true", "yes", "on"):
        imu_arguments.append("--print-debug")

    imu_node = Node(
        package="imu_cartographer_publisher",
        executable="imu_cartographer_publisher",
        name="imu_cartographer_publisher",
        output="screen",
        arguments=imu_arguments,
    )

    imu_tf_node = Node(
        package="imu_cartographer_publisher",
        executable="imu_orientation_tf",
        name="imu_orientation_tf",
        output="screen",
        parameters=[{
            "imu_topic": topic,
            "parent_frame": LaunchConfiguration("fixed_frame"),
            "child_frame": LaunchConfiguration("vehicle_frame"),
            "invert_orientation": LaunchConfiguration("invert_orientation"),
            "yaw_only": LaunchConfiguration("yaw_only"),
        }],
    )

    differential_drive_xacro = os.path.join(
        get_package_share_directory("jetacker_description"),
        "urdf",
        "differential_drive.urdf.xacro",
    )
    robot_description = Command(["xacro ", differential_drive_xacro])
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_robot_model")),
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="imu_orientation_rviz",
        output="screen",
        arguments=["-d", os.path.join(package_share, "rviz", "imu_orientation.rviz")],
    )

    return [imu_node, imu_tf_node, robot_state_publisher_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topic", default_value="/imu"),
        DeclareLaunchArgument("frame_id", default_value="imu_link"),
        DeclareLaunchArgument("i2c_bus", default_value="4"),
        DeclareLaunchArgument("device_addr", default_value="0x6A"),
        DeclareLaunchArgument("sample_period", default_value="0.08"),
        DeclareLaunchArgument(
            "axis_map",
            default_value="-y,-x,-z",
            description="Robot axes expressed as sensor axes: robot_x,robot_y,robot_z.",
        ),
        DeclareLaunchArgument("fixed_frame", default_value="world"),
        DeclareLaunchArgument(
            "vehicle_frame",
            default_value="base_footprint",
            description="TF frame rotated by IMU orientation. Use base_footprint for the differential chassis model.",
        ),
        DeclareLaunchArgument(
            "invert_orientation",
            default_value="false",
            description="Set true if RViz rotation is opposite to the real vehicle.",
        ),
        DeclareLaunchArgument(
            "yaw_only",
            default_value="false",
            description="Only apply IMU yaw to RViz TF; roll and pitch are forced to zero.",
        ),
        DeclareLaunchArgument(
            "use_robot_model",
            default_value="true",
            description="Show the differential chassis model in RViz when jetacker_description is available.",
        ),
        DeclareLaunchArgument(
            "print_debug",
            default_value="true",
            description="Print roll/pitch/yaw and raw IMU values while RViz is running.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
