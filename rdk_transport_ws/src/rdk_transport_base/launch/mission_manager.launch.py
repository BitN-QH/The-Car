from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("default_task_json", default_value="{}"),
        DeclareLaunchArgument("search_commands", default_value="rotate_left,rotate_left,stop"),
        DeclareLaunchArgument("approach_command", default_value="forward"),
        DeclareLaunchArgument("approach_cycles", default_value="3"),
        DeclareLaunchArgument("use_nav2_action", default_value="true"),
        Node(
            package="rdk_transport_base",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[{
                "default_task_json": ParameterValue(
                    LaunchConfiguration("default_task_json"),
                    value_type=str,
                ),
                "search_commands": LaunchConfiguration("search_commands"),
                "approach_command": LaunchConfiguration("approach_command"),
                "approach_cycles": ParameterValue(
                    LaunchConfiguration("approach_cycles"),
                    value_type=int,
                ),
                "use_nav2_action": ParameterValue(
                    LaunchConfiguration("use_nav2_action"),
                    value_type=bool,
                ),
            }],
        ),
    ])
