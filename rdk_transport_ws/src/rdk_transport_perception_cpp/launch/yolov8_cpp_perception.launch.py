import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    os.environ["CAM_TYPE"] = "usb"

    usb_video_device = LaunchConfiguration("usb_video_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    score_threshold = LaunchConfiguration("score_threshold")
    ai_topic = LaunchConfiguration("ai_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    launch_target_pose = LaunchConfiguration("launch_target_pose")

    hobot_usb_cam_launch = os.path.join(
        get_package_share_directory("hobot_usb_cam"),
        "launch",
        "hobot_usb_cam.launch.py",
    )
    hobot_codec_launch = os.path.join(
        get_package_share_directory("hobot_codec"),
        "launch",
        "hobot_codec_decode.launch.py",
    )
    hobot_shm_launch = os.path.join(
        get_package_share_directory("hobot_shm"),
        "launch",
        "hobot_shm.launch.py",
    )
    dnn_node_example_path = os.path.join(
        get_package_prefix("dnn_node_example"),
        "lib",
        "dnn_node_example",
    )

    return LaunchDescription([
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("config_file", default_value="config/yolov8workconfig.json"),
        DeclareLaunchArgument("score_threshold", default_value="0.35"),
        DeclareLaunchArgument(
            "model_path",
            default_value="/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin",
        ),
        DeclareLaunchArgument("ai_topic", default_value="/hobot_dnn_detection"),
        DeclareLaunchArgument("detections_topic", default_value="/perception/detections"),
        DeclareLaunchArgument("launch_target_pose", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hobot_shm_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hobot_usb_cam_launch),
            launch_arguments={
                "usb_video_device": usb_video_device,
                "usb_image_width": image_width,
                "usb_image_height": image_height,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hobot_codec_launch),
            launch_arguments={
                "codec_in_mode": "ros",
                "codec_out_mode": "shared_mem",
                "codec_sub_topic": "/image",
                "codec_pub_topic": "/hbmem_img",
            }.items(),
        ),
        Node(
            package="dnn_node_example",
            executable="example",
            output="screen",
            cwd=dnn_node_example_path,
            parameters=[{
                "config_file": config_file,
                "dump_render_img": 0,
                "feed_type": 1,
                "is_shared_mem_sub": 1,
                "msg_pub_topic_name": ai_topic,
            }],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
        Node(
            package="rdk_transport_perception_cpp",
            executable="ai_detections_json_bridge",
            name="ai_detections_json_bridge",
            output="screen",
            parameters=[{
                "input_topic": ai_topic,
                "output_topic": detections_topic,
                "model_path": model_path,
                "score_threshold": ParameterValue(score_threshold, value_type=float),
                "image_width": ParameterValue(image_width, value_type=int),
                "image_height": ParameterValue(image_height, value_type=int),
            }],
        ),
        Node(
            package="rdk_transport_base",
            executable="target_pose_node",
            name="target_pose_node",
            output="screen",
            condition=IfCondition(launch_target_pose),
            parameters=[{
                "detections_topic": detections_topic,
            }],
        ),
    ])
