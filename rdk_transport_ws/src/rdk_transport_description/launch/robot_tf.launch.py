import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory("rdk_transport_description")
    urdf_file = os.path.join(description_dir, "urdf", "rdk_transport.urdf")
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = {"robot_description": f.read()}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ])
