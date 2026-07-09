from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("imu_topic", default_value="/imu/data_raw"),
        DeclareLaunchArgument("odom_topic", default_value="/imu/odom"),
        DeclareLaunchArgument("odom_frame_id", default_value="imu_odom"),
        DeclareLaunchArgument("base_frame_id", default_value="base_link"),
        DeclareLaunchArgument("calibration_samples", default_value="200"),
        Node(
            package="rdk_transport_base",
            executable="h30_imu_odom_node",
            name="h30_imu_odom_node",
            output="screen",
            parameters=[{
                "imu_topic": LaunchConfiguration("imu_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "calibration_samples": LaunchConfiguration("calibration_samples"),
            }],
        ),
    ])
