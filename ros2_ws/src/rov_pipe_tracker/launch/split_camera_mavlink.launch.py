from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    command_topic = LaunchConfiguration("command_topic")
    target_ip = LaunchConfiguration("target_ip")
    target_port = LaunchConfiguration("target_port")
    udp_bind_port = LaunchConfiguration("udp_bind_port")
    learn_target_from_udp = LaunchConfiguration("learn_target_from_udp")
    set_manual_mode = LaunchConfiguration("set_manual_mode")
    auto_arm = LaunchConfiguration("auto_arm")
    manual_mode = LaunchConfiguration("manual_mode")
    startup_command_retries = LaunchConfiguration("startup_command_retries")
    startup_command_interval_s = LaunchConfiguration("startup_command_interval_s")
    command_timeout_s = LaunchConfiguration("command_timeout_s")

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("command_topic", default_value="mavlink_manual_control_command"),
        DeclareLaunchArgument("target_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("target_port", default_value="14550"),
        DeclareLaunchArgument("udp_bind_port", default_value="14550"),
        DeclareLaunchArgument("learn_target_from_udp", default_value="true"),
        DeclareLaunchArgument("set_manual_mode", default_value="true"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("manual_mode", default_value="19"),
        DeclareLaunchArgument("startup_command_retries", default_value="5"),
        DeclareLaunchArgument("startup_command_interval_s", default_value="0.5"),
        DeclareLaunchArgument("command_timeout_s", default_value="0.5"),
        ComposableNodeContainer(
            name="split_camera_mavlink_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="camera_driver",
                    plugin="camera_driver::V4L2CameraNode",
                    name="v4l2_camera",
                    parameters=[{
                        "device": device,
                        "width": width,
                        "height": height,
                        "fps": fps,
                        "topic": "image_raw",
                    }],
                    extra_arguments=[{"use_intra_process_comms": False}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::MavlinkManualControlBridgeNode",
                    name="mavlink_manual_control_bridge",
                    parameters=[{
                        "command_topic": command_topic,
                        "target_ip": target_ip,
                        "target_port": target_port,
                        "udp_bind_port": udp_bind_port,
                        "learn_target_from_udp": learn_target_from_udp,
                        "set_manual_mode": set_manual_mode,
                        "auto_arm": auto_arm,
                        "manual_mode": manual_mode,
                        "startup_command_retries": startup_command_retries,
                        "startup_command_interval_s": startup_command_interval_s,
                        "command_timeout_s": command_timeout_s,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        ),
    ])
