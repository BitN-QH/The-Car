from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("route", default_value="forward,left,rotate_right,forward,stop"),
        DeclareLaunchArgument("segment_duration_sec", default_value="0.35"),
        DeclareLaunchArgument("settle_duration_sec", default_value="0.8"),
        Node(
            package="rdk_transport_base",
            executable="segmented_nav_node",
            name="segmented_nav_node",
            output="screen",
            parameters=[{
                "route": LaunchConfiguration("route"),
                "segment_duration_sec": LaunchConfiguration("segment_duration_sec"),
                "settle_duration_sec": LaunchConfiguration("settle_duration_sec"),
            }],
        ),
    ])
