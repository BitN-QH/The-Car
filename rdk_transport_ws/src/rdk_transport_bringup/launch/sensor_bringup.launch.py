import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_usb_camera = LaunchConfiguration("launch_usb_camera")
    usb_video_device = LaunchConfiguration("usb_video_device")
    usb_image_width = LaunchConfiguration("usb_image_width")
    usb_image_height = LaunchConfiguration("usb_image_height")
    usb_framerate = LaunchConfiguration("usb_framerate")
    usb_pixel_format = LaunchConfiguration("usb_pixel_format")
    usb_zero_copy = LaunchConfiguration("usb_zero_copy")

    lidar_launch = "/home/sunrise/lslidar_ws/src/lslidar_driver/launch/lsn10_launch.py"
    imu_launch = "/home/sunrise/h30_imu_ws/src/yesense_ros2/yesense_std_ros2/launch/yesense_node.launch.py"

    return LaunchDescription([
        DeclareLaunchArgument("launch_lidar", default_value="true"),
        DeclareLaunchArgument("launch_imu", default_value="true"),
        DeclareLaunchArgument("launch_usb_camera", default_value="true"),
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video2"),
        DeclareLaunchArgument("usb_image_width", default_value="640"),
        DeclareLaunchArgument("usb_image_height", default_value="480"),
        DeclareLaunchArgument("usb_framerate", default_value="30"),
        DeclareLaunchArgument("usb_pixel_format", default_value="yuyv2rgb"),
        DeclareLaunchArgument("usb_zero_copy", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(launch_lidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            condition=IfCondition(launch_imu),
        ),
        Node(
            package="hobot_usb_cam",
            executable="hobot_usb_cam",
            name="usb_cam",
            output="screen",
            condition=IfCondition(launch_usb_camera),
            parameters=[{
                "usb_video_device": usb_video_device,
                "usb_image_width": ParameterValue(usb_image_width, value_type=int),
                "usb_image_height": ParameterValue(usb_image_height, value_type=int),
                "usb_framerate": ParameterValue(usb_framerate, value_type=int),
                "usb_pixel_format": usb_pixel_format,
                "usb_zero_copy": ParameterValue(usb_zero_copy, value_type=bool),
            }],
        ),
    ])
