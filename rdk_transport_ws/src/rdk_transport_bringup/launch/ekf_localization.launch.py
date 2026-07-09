from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution([
        FindPackageShare("rdk_transport_bringup"),
        "config",
        "ekf_odom_imu.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[params_file],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),
    ])
