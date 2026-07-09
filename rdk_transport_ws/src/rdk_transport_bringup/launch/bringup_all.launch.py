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
        DeclareLaunchArgument("launch_lidar", default_value="true"),
        DeclareLaunchArgument("launch_imu", default_value="true"),
        DeclareLaunchArgument("launch_usb_camera", default_value="true"),
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video2"),
        include_launch("rdk_transport_description", "robot_tf.launch.py"),
        include_launch("rdk_transport_base", "base_mock.launch.py"),
        include_launch("rdk_transport_base", "manual_command.launch.py"),
        include_launch(
            "rdk_transport_bringup",
            "sensor_bringup.launch.py",
            {
                "launch_lidar": LaunchConfiguration("launch_lidar"),
                "launch_imu": LaunchConfiguration("launch_imu"),
                "launch_usb_camera": LaunchConfiguration("launch_usb_camera"),
                "usb_video_device": LaunchConfiguration("usb_video_device"),
            },
        ),
    ])
