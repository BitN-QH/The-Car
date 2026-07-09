from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution([
        FindPackageShare("rdk_transport_bringup"),
        "config",
        "slam_toolbox_mapping_imu.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("rdk_transport_base"),
                    "launch",
                    "h30_imu_odom.launch.py",
                ])
            ),
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="sync_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[params_file],
                ),
            ],
        ),
    ])
