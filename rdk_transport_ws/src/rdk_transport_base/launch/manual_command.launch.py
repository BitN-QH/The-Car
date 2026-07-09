from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rdk_transport_base",
            executable="manual_command_node",
            name="manual_command_node",
            output="screen",
        ),
    ])
