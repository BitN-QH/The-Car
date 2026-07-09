from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    slam_executable = LaunchConfiguration("slam_executable")

    default_params = PathJoinSubstitution([
        FindPackageShare("rdk_transport_bringup"),
        "config",
        "slam_toolbox_mapping.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("slam_executable", default_value="sync_slam_toolbox_node"),
        Node(
            package="slam_toolbox",
            executable=slam_executable,
            name="slam_toolbox",
            output="screen",
            parameters=[params_file],
        ),
    ])
