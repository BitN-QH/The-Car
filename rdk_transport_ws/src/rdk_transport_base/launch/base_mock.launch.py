from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("odom_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.5"),
        Node(
            package="rdk_transport_base",
            executable="base_bridge_node",
            name="base_bridge_node",
            output="screen",
            parameters=[{
                "mock_mode": True,
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "cmd_vel_topic": "/cmd_vel",
                "odom_topic": "/odom",
                "publish_tf": True,
                "odom_rate_hz": LaunchConfiguration("odom_rate_hz"),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
            }],
        ),
    ])
