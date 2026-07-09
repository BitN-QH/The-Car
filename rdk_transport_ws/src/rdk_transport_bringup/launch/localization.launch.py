from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_file_name = LaunchConfiguration("map_file_name")

    default_params = PathJoinSubstitution([
        FindPackageShare("rdk_transport_bringup"),
        "config",
        "slam_toolbox_localization.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "map_file_name",
            default_value="/home/sunrise/rdk_transport_ws/maps/rdk_x5_demo_map",
        ),
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[params_file, {"map_file_name": map_file_name}],
        ),
    ])
