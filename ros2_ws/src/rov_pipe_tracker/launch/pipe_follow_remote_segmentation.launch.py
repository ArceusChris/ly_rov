from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    mask_topic = LaunchConfiguration("mask_topic")
    mask_stream_url = LaunchConfiguration("mask_stream_url")
    overlay_stream_url = LaunchConfiguration("overlay_stream_url")
    processed_stream_url = LaunchConfiguration("processed_stream_url")
    target_ip = LaunchConfiguration("target_ip")
    target_port = LaunchConfiguration("target_port")
    lateral_sign = LaunchConfiguration("lateral_sign")
    yaw_sign = LaunchConfiguration("yaw_sign")
    desired_angle_deg = LaunchConfiguration("desired_angle_deg")
    forward_axis = LaunchConfiguration("forward_axis")
    set_manual_mode = LaunchConfiguration("set_manual_mode")
    auto_arm = LaunchConfiguration("auto_arm")
    manual_mode = LaunchConfiguration("manual_mode")
    startup_command_retries = LaunchConfiguration("startup_command_retries")
    startup_command_interval_s = LaunchConfiguration("startup_command_interval_s")
    manual_control_enabled = LaunchConfiguration("manual_control_enabled")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    web_jpeg_quality = LaunchConfiguration("web_jpeg_quality")
    web_stream_fps = LaunchConfiguration("web_stream_fps")
    manual_web_host = LaunchConfiguration("manual_web_host")
    manual_web_port = LaunchConfiguration("manual_web_port")
    manual_command_topic = LaunchConfiguration("manual_command_topic")
    command_log_topic = LaunchConfiguration("command_log_topic")

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("mask_topic", default_value="remote_pipe_segmentation"),
        DeclareLaunchArgument("mask_stream_url", default_value="http://192.168.127.11:8090/mask.mjpg"),
        DeclareLaunchArgument(
            "overlay_stream_url",
            default_value="http://192.168.127.11:8090/processed.mjpg",
        ),
        DeclareLaunchArgument(
            "processed_stream_url",
            default_value="http://192.168.127.11:8090/mask.mjpg",
        ),
        DeclareLaunchArgument("target_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("target_port", default_value="14550"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="150"),
        DeclareLaunchArgument("set_manual_mode", default_value="true"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("manual_mode", default_value="19"),
        DeclareLaunchArgument("startup_command_retries", default_value="5"),
        DeclareLaunchArgument("startup_command_interval_s", default_value="0.5"),
        DeclareLaunchArgument("manual_control_enabled", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("web_jpeg_quality", default_value="80"),
        DeclareLaunchArgument("web_stream_fps", default_value="15.0"),
        DeclareLaunchArgument("manual_web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("manual_web_port", default_value="8081"),
        DeclareLaunchArgument("manual_command_topic", default_value="manual_control_command"),
        DeclareLaunchArgument("command_log_topic", default_value="pipe_tracker_command_log"),
        Node(
            package="rov_pipe_tracker",
            executable="remote_mask_mjpeg_bridge.py",
            name="remote_mask_mjpeg_bridge",
            parameters=[{
                "input_url": mask_stream_url,
                "output_topic": mask_topic,
            }],
            output="screen",
        ),
        ComposableNodeContainer(
            name="pipe_follow_remote_segmentation_container",
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
                    plugin="rov_pipe_tracker::PipeTrackerNode",
                    name="pipe_tracker",
                    parameters=[{
                        "mask_topic": mask_topic,
                        "manual_command_topic": manual_command_topic,
                        "command_log_topic": command_log_topic,
                        "target_ip": target_ip,
                        "target_port": target_port,
                        "udp_bind_port": target_port,
                        "learn_target_from_udp": True,
                        "lateral_sign": lateral_sign,
                        "yaw_sign": yaw_sign,
                        "desired_angle_deg": desired_angle_deg,
                        "forward_axis": forward_axis,
                        "set_manual_mode": set_manual_mode,
                        "auto_arm": auto_arm,
                        "manual_mode": manual_mode,
                        "startup_command_retries": startup_command_retries,
                        "startup_command_interval_s": startup_command_interval_s,
                        "manual_control_enabled": manual_control_enabled,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::PipeWebUiNode",
                    name="pipe_web_ui",
                    parameters=[{
                        "raw_topic": "image_raw",
                        "mask_topic": mask_topic,
                        "processed_topic": "",
                        "overlay_stream_url": overlay_stream_url,
                        "processed_stream_url": processed_stream_url,
                        "command_log_topic": command_log_topic,
                        "http_host": web_host,
                        "http_port": web_port,
                        "jpeg_quality": web_jpeg_quality,
                        "stream_fps": web_stream_fps,
                        "arm_service": "/pipe_tracker/arm",
                        "manual_mode_service": "/pipe_tracker/manual_mode",
                        "manual_control_service": "/pipe_tracker/manual_control_enabled",
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::ManualControlWebUiNode",
                    name="manual_control_web_ui",
                    parameters=[{
                        "command_topic": manual_command_topic,
                        "http_host": manual_web_host,
                        "http_port": manual_web_port,
                        "arm_service": "/pipe_tracker/arm",
                        "manual_mode_service": "/pipe_tracker/manual_mode",
                        "manual_control_service": "/pipe_tracker/manual_control_enabled",
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        ),
    ])
