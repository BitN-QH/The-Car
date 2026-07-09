from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cpp_launch = os.path.join(
        get_package_share_directory("rdk_transport_perception_cpp"),
        "launch",
        "yolov8_cpp_perception.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("score_threshold", default_value="0.35"),
        DeclareLaunchArgument("launch_target_pose", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cpp_launch),
            launch_arguments={
                "usb_video_device": LaunchConfiguration("usb_video_device"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "score_threshold": LaunchConfiguration("score_threshold"),
                "launch_target_pose": LaunchConfiguration("launch_target_pose"),
            }.items(),
        ),
    ])
