from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    raw_image_topic = LaunchConfiguration("raw_image_topic")
    image_topic = LaunchConfiguration("image_topic")
    mask_topic = LaunchConfiguration("mask_topic")
    command_topic = LaunchConfiguration("command_topic")
    lateral_sign = LaunchConfiguration("lateral_sign")
    yaw_sign = LaunchConfiguration("yaw_sign")
    desired_angle_deg = LaunchConfiguration("desired_angle_deg")
    forward_axis = LaunchConfiguration("forward_axis")
    manual_control_enabled = LaunchConfiguration("manual_control_enabled")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    web_jpeg_quality = LaunchConfiguration("web_jpeg_quality")
    web_stream_fps = LaunchConfiguration("web_stream_fps")
    manual_web_host = LaunchConfiguration("manual_web_host")
    manual_web_port = LaunchConfiguration("manual_web_port")
    tuning_web_host = LaunchConfiguration("tuning_web_host")
    tuning_web_port = LaunchConfiguration("tuning_web_port")
    manual_command_topic = LaunchConfiguration("manual_command_topic")
    command_log_topic = LaunchConfiguration("command_log_topic")
    bridge_node_name = LaunchConfiguration("bridge_node_name")
    bridge_arm_service = PythonExpression(["'/' + '", bridge_node_name, "' + '/arm'"])
    bridge_manual_mode_service = PythonExpression(["'/' + '", bridge_node_name, "' + '/manual_mode'"])
    bridge_manual_control_service = PythonExpression(
        ["'/' + '", bridge_node_name, "' + '/manual_control_enabled'"])

    return LaunchDescription([
        DeclareLaunchArgument("raw_image_topic", default_value="image_raw"),
        DeclareLaunchArgument("image_topic", default_value="image_dehazed"),
        DeclareLaunchArgument("mask_topic", default_value="pipe_cv_segmentation"),
        DeclareLaunchArgument("command_topic", default_value="mavlink_manual_control_command"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="150"),
        DeclareLaunchArgument("manual_control_enabled", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("web_jpeg_quality", default_value="80"),
        DeclareLaunchArgument("web_stream_fps", default_value="15.0"),
        DeclareLaunchArgument("manual_web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("manual_web_port", default_value="8081"),
        DeclareLaunchArgument("tuning_web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("tuning_web_port", default_value="8082"),
        DeclareLaunchArgument("manual_command_topic", default_value="manual_control_command"),
        DeclareLaunchArgument("command_log_topic", default_value="pipe_tracker_command_log"),
        DeclareLaunchArgument("bridge_node_name", default_value="mavlink_manual_control_bridge"),
        DeclareLaunchArgument("min_red", default_value="150"),
        DeclareLaunchArgument("min_green", default_value="150"),
        DeclareLaunchArgument("min_blue", default_value="150"),
        DeclareLaunchArgument("max_channel_diff", default_value="45"),
        DeclareLaunchArgument("max_red_blue_diff", default_value="55"),
        DeclareLaunchArgument("min_area", default_value="3000"),
        DeclareLaunchArgument("min_height_ratio", default_value="0.18"),
        DeclareLaunchArgument("min_aspect", default_value="0.5"),
        DeclareLaunchArgument("max_aspect", default_value="2.0"),
        DeclareLaunchArgument("erode_kernel_size", default_value="1"),
        ComposableNodeContainer(
            name="split_processing_cv_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="dehaze",
                    plugin="dehaze::UDCPDehazeNode",
                    name="udcp_dehaze",
                    parameters=[{
                        "input_topic": raw_image_topic,
                        "output_topic": image_topic,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::CvPipeSegmenterNode",
                    name="cv_pipe_segmenter",
                    parameters=[{
                        "input_topic": image_topic,
                        "output_topic": mask_topic,
                        "min_red": LaunchConfiguration("min_red"),
                        "min_green": LaunchConfiguration("min_green"),
                        "min_blue": LaunchConfiguration("min_blue"),
                        "max_channel_diff": LaunchConfiguration("max_channel_diff"),
                        "max_red_blue_diff": LaunchConfiguration("max_red_blue_diff"),
                        "min_area": LaunchConfiguration("min_area"),
                        "min_height_ratio": LaunchConfiguration("min_height_ratio"),
                        "min_aspect": LaunchConfiguration("min_aspect"),
                        "max_aspect": LaunchConfiguration("max_aspect"),
                        "erode_kernel_size": LaunchConfiguration("erode_kernel_size"),
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::PipeTrackerNode",
                    name="pipe_tracker",
                    parameters=[{
                        "mask_topic": mask_topic,
                        "manual_command_topic": manual_command_topic,
                        "command_log_topic": command_log_topic,
                        "output_command_topic": command_topic,
                        "mavlink_enabled": False,
                        "lateral_sign": lateral_sign,
                        "yaw_sign": yaw_sign,
                        "desired_angle_deg": desired_angle_deg,
                        "forward_axis": forward_axis,
                        "manual_control_enabled": manual_control_enabled,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::PipeWebUiNode",
                    name="pipe_web_ui",
                    parameters=[{
                        "raw_topic": image_topic,
                        "mask_topic": mask_topic,
                        "command_log_topic": command_log_topic,
                        "http_host": web_host,
                        "http_port": web_port,
                        "jpeg_quality": web_jpeg_quality,
                        "stream_fps": web_stream_fps,
                        "arm_service": bridge_arm_service,
                        "manual_mode_service": bridge_manual_mode_service,
                        "manual_control_service": bridge_manual_control_service,
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
                        "arm_service": bridge_arm_service,
                        "manual_mode_service": bridge_manual_mode_service,
                        "manual_control_service": bridge_manual_control_service,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::ParameterTuningWebUiNode",
                    name="parameter_tuning_web_ui",
                    parameters=[{
                        "http_host": tuning_web_host,
                        "http_port": tuning_web_port,
                        "segmenter_node": "/cv_pipe_segmenter",
                        "tracker_node": "/pipe_tracker",
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        ),
    ])
