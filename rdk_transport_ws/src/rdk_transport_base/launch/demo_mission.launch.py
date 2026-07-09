from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("step_interval_sec", default_value="1.5"),
        Node(
            package="rdk_transport_base",
            executable="demo_mission_node",
            name="demo_mission_node",
            output="screen",
            parameters=[{
                "step_interval_sec": LaunchConfiguration("step_interval_sec"),
            }],
        ),
    ])
