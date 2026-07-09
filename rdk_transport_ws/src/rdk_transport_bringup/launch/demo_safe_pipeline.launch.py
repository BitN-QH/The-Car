from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                "launch",
                launch_file,
            ])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_lidar", default_value="false"),
        DeclareLaunchArgument("launch_imu", default_value="false"),
        DeclareLaunchArgument("launch_usb_camera", default_value="false"),
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video2"),
        DeclareLaunchArgument("route", default_value="forward,left,rotate_right,forward,stop"),
        include_launch("rdk_transport_bringup", "bringup_all.launch.py", {
            "launch_lidar": LaunchConfiguration("launch_lidar"),
            "launch_imu": LaunchConfiguration("launch_imu"),
            "launch_usb_camera": LaunchConfiguration("launch_usb_camera"),
            "usb_video_device": LaunchConfiguration("usb_video_device"),
        }),
        include_launch("rdk_transport_base", "segmented_nav.launch.py", {
            "route": LaunchConfiguration("route"),
        }),
    ])
