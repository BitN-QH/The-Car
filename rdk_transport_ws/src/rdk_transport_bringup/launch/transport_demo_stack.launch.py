from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, launch_file, launch_arguments=None, condition=None):
    kwargs = {}
    if condition is not None:
        kwargs["condition"] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                "launch",
                launch_file,
            ])
        ),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


def generate_launch_description():
    use_serial_base = LaunchConfiguration("use_serial_base")
    launch_sensors = LaunchConfiguration("launch_sensors")
    launch_slam = LaunchConfiguration("launch_slam")
    launch_yolo = LaunchConfiguration("launch_yolo")
    launch_web = LaunchConfiguration("launch_web")
    launch_mission = LaunchConfiguration("launch_mission")

    return LaunchDescription([
        DeclareLaunchArgument("use_serial_base", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value=""),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("serial_write_commands", default_value="false"),
        DeclareLaunchArgument("allow_protected_serial", default_value="false"),
        DeclareLaunchArgument("launch_sensors", default_value="false"),
        DeclareLaunchArgument("launch_lidar", default_value="true"),
        DeclareLaunchArgument("launch_imu", default_value="false"),
        DeclareLaunchArgument("launch_usb_camera", default_value="false"),
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("launch_slam", default_value="false"),
        DeclareLaunchArgument("launch_yolo", default_value="false"),
        DeclareLaunchArgument("launch_web", default_value="true"),
        DeclareLaunchArgument("launch_mission", default_value="true"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("default_task_json", default_value="{}"),

        include_launch("rdk_transport_description", "robot_tf.launch.py"),
        include_launch("rdk_transport_base", "manual_command.launch.py"),
        include_launch(
            "rdk_transport_base",
            "base_mock.launch.py",
            condition=UnlessCondition(use_serial_base),
        ),
        include_launch(
            "rdk_transport_base",
            "base_serial.launch.py",
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": LaunchConfiguration("serial_baudrate"),
                "serial_write_commands": LaunchConfiguration("serial_write_commands"),
                "allow_protected_serial": LaunchConfiguration("allow_protected_serial"),
            },
            condition=IfCondition(use_serial_base),
        ),
        include_launch(
            "rdk_transport_bringup",
            "sensor_bringup.launch.py",
            {
                "launch_lidar": LaunchConfiguration("launch_lidar"),
                "launch_imu": LaunchConfiguration("launch_imu"),
                "launch_usb_camera": LaunchConfiguration("launch_usb_camera"),
                "usb_video_device": LaunchConfiguration("usb_video_device"),
            },
            condition=IfCondition(launch_sensors),
        ),
        include_launch("rdk_transport_base", "scan_sanitizer.launch.py"),
        include_launch(
            "rdk_transport_bringup",
            "slam_mapping.launch.py",
            condition=IfCondition(launch_slam),
        ),
        include_launch(
            "rdk_transport_base",
            "perception_stub.launch.py",
            {
                "usb_video_device": LaunchConfiguration("usb_video_device"),
                "launch_target_pose": "true",
            },
            condition=IfCondition(launch_yolo),
        ),
        include_launch(
            "rdk_transport_base",
            "yolo_web_monitor.launch.py",
            {"port": LaunchConfiguration("web_port")},
            condition=IfCondition(launch_web),
        ),
        include_launch(
            "rdk_transport_base",
            "mission_manager.launch.py",
            {"default_task_json": LaunchConfiguration("default_task_json")},
            condition=IfCondition(launch_mission),
        ),
    ])
