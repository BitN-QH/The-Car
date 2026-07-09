from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/scan"),
        DeclareLaunchArgument("output_topic", default_value="/scan_slam"),
        DeclareLaunchArgument("range_max", default_value="12.0"),
        Node(
            package="rdk_transport_base",
            executable="scan_sanitizer_node",
            name="scan_sanitizer_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "range_max": LaunchConfiguration("range_max"),
            }],
        ),
    ])
