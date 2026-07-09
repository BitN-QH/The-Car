import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration("model").perform(context)
    with open(model_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="wheelbot_reference_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ]


def generate_launch_description():
    description_dir = get_package_share_directory("rdk_transport_description")
    default_model = os.path.join(
        description_dir,
        "urdf",
        "wheelbot_reference",
        "wheelbot_reference.urdf",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=default_model,
            description="Wheelbot reference URDF model.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
