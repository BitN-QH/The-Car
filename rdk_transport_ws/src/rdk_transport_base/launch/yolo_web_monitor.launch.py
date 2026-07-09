from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("detections_topic", default_value="/perception/detections"),
        DeclareLaunchArgument("terminal_cmd_topic", default_value="/mission/terminal_cmd"),
        DeclareLaunchArgument("state_topic", default_value="/mission/state"),
        DeclareLaunchArgument("image_topic", default_value=""),
        DeclareLaunchArgument("compressed_image_topic", default_value="/image"),
        DeclareLaunchArgument("aux_image_topic", default_value=""),
        DeclareLaunchArgument("aux_compressed_image_topic", default_value="/image_aux"),
        DeclareLaunchArgument("aux_video_device", default_value="/dev/video2"),
        DeclareLaunchArgument("aux_video_width", default_value="320"),
        DeclareLaunchArgument("aux_video_height", default_value="240"),
        DeclareLaunchArgument("aux_video_fps", default_value="30"),
        DeclareLaunchArgument("aux_raw_publish_topic", default_value="/image_aux_raw"),
        DeclareLaunchArgument("yolo_source_script", default_value="/tmp/select_yolo_source.sh"),
        DeclareLaunchArgument("model_http_url", default_value="http://127.0.0.1:18789/hooks/agent"),
        DeclareLaunchArgument("model_http_token", default_value=""),
        DeclareLaunchArgument("model_http_token_file", default_value=""),
        DeclareLaunchArgument("model_http_fallback_cli", default_value="true"),
        DeclareLaunchArgument("model_transcript_sessions_json", default_value="/root/.openclaw/agents/main/sessions/sessions.json"),
        DeclareLaunchArgument("model_sudo_password", default_value="sunrise"),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        ),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("center_deadband_ratio", default_value="0.12"),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8080"),
        Node(
            package="rdk_transport_base",
            executable="yolo_web_monitor_node",
            name="yolo_web_monitor_node",
            output="screen",
            parameters=[{
                "detections_topic": LaunchConfiguration("detections_topic"),
                "terminal_cmd_topic": LaunchConfiguration("terminal_cmd_topic"),
                "state_topic": LaunchConfiguration("state_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "compressed_image_topic": LaunchConfiguration("compressed_image_topic"),
                "aux_image_topic": LaunchConfiguration("aux_image_topic"),
                "aux_compressed_image_topic": LaunchConfiguration("aux_compressed_image_topic"),
                "aux_video_device": LaunchConfiguration("aux_video_device"),
                "aux_video_width": ParameterValue(
                    LaunchConfiguration("aux_video_width"),
                    value_type=int,
                ),
                "aux_video_height": ParameterValue(
                    LaunchConfiguration("aux_video_height"),
                    value_type=int,
                ),
                "aux_video_fps": ParameterValue(
                    LaunchConfiguration("aux_video_fps"),
                    value_type=int,
                ),
                "aux_raw_publish_topic": LaunchConfiguration("aux_raw_publish_topic"),
                "yolo_source_script": LaunchConfiguration("yolo_source_script"),
                "model_http_url": LaunchConfiguration("model_http_url"),
                "model_http_token": LaunchConfiguration("model_http_token"),
                "model_http_token_file": LaunchConfiguration("model_http_token_file"),
                "model_http_fallback_cli": ParameterValue(
                    LaunchConfiguration("model_http_fallback_cli"),
                    value_type=bool,
                ),
                "model_transcript_sessions_json": LaunchConfiguration("model_transcript_sessions_json"),
                "model_sudo_password": LaunchConfiguration("model_sudo_password"),
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": ParameterValue(
                    LaunchConfiguration("serial_baudrate"),
                    value_type=int,
                ),
                "center_deadband_ratio": ParameterValue(
                    LaunchConfiguration("center_deadband_ratio"),
                    value_type=float,
                ),
                "host": LaunchConfiguration("host"),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
            }],
        ),
    ])
