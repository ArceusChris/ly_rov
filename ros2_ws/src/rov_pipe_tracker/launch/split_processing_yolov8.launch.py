import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    os.chdir(get_package_share_directory("dehaze_segmentation"))

    raw_image_topic = LaunchConfiguration("raw_image_topic")
    image_topic = LaunchConfiguration("image_topic")
    mask_topic = LaunchConfiguration("mask_topic")
    command_topic = LaunchConfiguration("command_topic")
    config_file = LaunchConfiguration("config_file")
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
        DeclareLaunchArgument("mask_topic", default_value="hobot_dnn_segmentation"),
        DeclareLaunchArgument("command_topic", default_value="mavlink_manual_control_command"),
        DeclareLaunchArgument("config_file", default_value="config/yolov8segworkconfig.json"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="250"),
        DeclareLaunchArgument("manual_control_enabled", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("web_jpeg_quality", default_value="80"),
        DeclareLaunchArgument("web_stream_fps", default_value="15.0"),
        DeclareLaunchArgument("manual_web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("manual_web_port", default_value="8081"),
        DeclareLaunchArgument("manual_command_topic", default_value="manual_control_command"),
        DeclareLaunchArgument("command_log_topic", default_value="pipe_tracker_command_log"),
        DeclareLaunchArgument("bridge_node_name", default_value="mavlink_manual_control_bridge"),
        ComposableNodeContainer(
            name="split_processing_yolov8_container",
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
                    package="dnn_node_example",
                    plugin="DnnExampleNode",
                    name="yolov8seg",
                    parameters=[{
                        "feed_type": 1,
                        "is_shared_mem_sub": 0,
                        "ros_img_topic_name": image_topic,
                        "config_file": config_file,
                        "msg_pub_topic_name": mask_topic,
                        "dump_render_img": 0,
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
            ],
            output="screen",
        ),
    ])
