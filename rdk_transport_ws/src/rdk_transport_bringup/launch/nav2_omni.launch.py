from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")

    default_params = PathJoinSubstitution([
        FindPackageShare("rdk_transport_bringup"),
        "config",
        "nav2_omni_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "map",
            default_value="/home/sunrise/rdk_transport_ws/maps/rdk_x5_demo_map.yaml",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ])
            ),
            launch_arguments={
                "slam": "False",
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": "False",
                "autostart": "True",
            }.items(),
        ),
    ])
