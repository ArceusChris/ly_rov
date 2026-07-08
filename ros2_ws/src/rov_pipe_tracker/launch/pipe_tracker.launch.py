from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_ip", default_value="192.168.2.2"),
        DeclareLaunchArgument("target_port", default_value="14550"),
        DeclareLaunchArgument("mask_topic", default_value="hobot_dnn_segmentation"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="250"),
        Node(
            package="rov_pipe_tracker",
            executable="pipe_tracker_node",
            name="pipe_tracker",
            output="screen",
            parameters=[{
                "target_ip": LaunchConfiguration("target_ip"),
                "target_port": LaunchConfiguration("target_port"),
                "mask_topic": LaunchConfiguration("mask_topic"),
                "lateral_sign": LaunchConfiguration("lateral_sign"),
                "yaw_sign": LaunchConfiguration("yaw_sign"),
                "desired_angle_deg": LaunchConfiguration("desired_angle_deg"),
                "forward_axis": LaunchConfiguration("forward_axis"),
            }],
        ),
    ])
