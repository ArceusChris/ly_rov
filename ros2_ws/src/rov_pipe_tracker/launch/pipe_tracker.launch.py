from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_ip", default_value="192.168.2.2"),
        DeclareLaunchArgument("target_port", default_value="14550"),
        DeclareLaunchArgument("mask_topic", default_value="hobot_dnn_segmentation"),
        DeclareLaunchArgument("manual_command_topic", default_value="manual_control_command"),
        DeclareLaunchArgument("command_log_topic", default_value="pipe_tracker_command_log"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="250"),
        DeclareLaunchArgument("set_manual_mode", default_value="true"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("manual_mode", default_value="19"),
        DeclareLaunchArgument("startup_command_retries", default_value="5"),
        DeclareLaunchArgument("startup_command_interval_s", default_value="0.5"),
        Node(
            package="rov_pipe_tracker",
            executable="pipe_tracker_node",
            name="pipe_tracker",
            output="screen",
            parameters=[{
                "target_ip": LaunchConfiguration("target_ip"),
                "target_port": LaunchConfiguration("target_port"),
                "mask_topic": LaunchConfiguration("mask_topic"),
                "manual_command_topic": LaunchConfiguration("manual_command_topic"),
                "command_log_topic": LaunchConfiguration("command_log_topic"),
                "lateral_sign": LaunchConfiguration("lateral_sign"),
                "yaw_sign": LaunchConfiguration("yaw_sign"),
                "desired_angle_deg": LaunchConfiguration("desired_angle_deg"),
                "forward_axis": LaunchConfiguration("forward_axis"),
                "set_manual_mode": LaunchConfiguration("set_manual_mode"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "manual_mode": LaunchConfiguration("manual_mode"),
                "startup_command_retries": LaunchConfiguration("startup_command_retries"),
                "startup_command_interval_s": LaunchConfiguration("startup_command_interval_s"),
            }],
        ),
    ])
