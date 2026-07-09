from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("detections_topic", default_value="/perception/detections"),
        DeclareLaunchArgument("manual_cmd_topic", default_value="/manual_cmd"),
        DeclareLaunchArgument("terminal_cmd_topic", default_value="/mission/terminal_cmd"),
        DeclareLaunchArgument("score_threshold", default_value="0.35"),
        DeclareLaunchArgument("control_period_sec", default_value="0.25"),
        DeclareLaunchArgument("center_deadband_ratio", default_value="0.12"),
        DeclareLaunchArgument("require_centered_to_advance", default_value="true"),
        DeclareLaunchArgument("close_height_ratio", default_value="0.72"),
        DeclareLaunchArgument("close_area_ratio", default_value="0.30"),
        DeclareLaunchArgument("target_height_ratio", default_value="0.62"),
        DeclareLaunchArgument("target_area_ratio", default_value="0.22"),
        DeclareLaunchArgument("size_deadband_ratio", default_value="0.08"),
        DeclareLaunchArgument("close_distance_m", default_value="0.75"),
        DeclareLaunchArgument("rotate_180_sec", default_value="3.2"),
        DeclareLaunchArgument("min_return_sec", default_value="1.0"),
        DeclareLaunchArgument("max_return_sec", default_value="8.0"),
        DeclareLaunchArgument("lost_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("publish_manual_cmd", default_value="true"),
        DeclareLaunchArgument("repeat_same_terminal_cmd", default_value="false"),
        Node(
            package="rdk_transport_base",
            executable="person_follow_mission_node",
            name="person_follow_mission_node",
            output="screen",
            parameters=[{
                "detections_topic": LaunchConfiguration("detections_topic"),
                "manual_cmd_topic": LaunchConfiguration("manual_cmd_topic"),
                "terminal_cmd_topic": LaunchConfiguration("terminal_cmd_topic"),
                "score_threshold": ParameterValue(
                    LaunchConfiguration("score_threshold"),
                    value_type=float,
                ),
                "control_period_sec": ParameterValue(
                    LaunchConfiguration("control_period_sec"),
                    value_type=float,
                ),
                "center_deadband_ratio": ParameterValue(
                    LaunchConfiguration("center_deadband_ratio"),
                    value_type=float,
                ),
                "require_centered_to_advance": ParameterValue(
                    LaunchConfiguration("require_centered_to_advance"),
                    value_type=bool,
                ),
                "close_height_ratio": ParameterValue(
                    LaunchConfiguration("close_height_ratio"),
                    value_type=float,
                ),
                "close_area_ratio": ParameterValue(
                    LaunchConfiguration("close_area_ratio"),
                    value_type=float,
                ),
                "target_height_ratio": ParameterValue(
                    LaunchConfiguration("target_height_ratio"),
                    value_type=float,
                ),
                "target_area_ratio": ParameterValue(
                    LaunchConfiguration("target_area_ratio"),
                    value_type=float,
                ),
                "size_deadband_ratio": ParameterValue(
                    LaunchConfiguration("size_deadband_ratio"),
                    value_type=float,
                ),
                "close_distance_m": ParameterValue(
                    LaunchConfiguration("close_distance_m"),
                    value_type=float,
                ),
                "rotate_180_sec": ParameterValue(
                    LaunchConfiguration("rotate_180_sec"),
                    value_type=float,
                ),
                "min_return_sec": ParameterValue(
                    LaunchConfiguration("min_return_sec"),
                    value_type=float,
                ),
                "max_return_sec": ParameterValue(
                    LaunchConfiguration("max_return_sec"),
                    value_type=float,
                ),
                "lost_timeout_sec": ParameterValue(
                    LaunchConfiguration("lost_timeout_sec"),
                    value_type=float,
                ),
                "publish_manual_cmd": ParameterValue(
                    LaunchConfiguration("publish_manual_cmd"),
                    value_type=bool,
                ),
                "repeat_same_terminal_cmd": ParameterValue(
                    LaunchConfiguration("repeat_same_terminal_cmd"),
                    value_type=bool,
                ),
            }],
        ),
    ])
