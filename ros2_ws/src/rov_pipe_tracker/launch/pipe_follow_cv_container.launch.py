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
    mask_topic = LaunchConfiguration("mask_topic")
    target_ip = LaunchConfiguration("target_ip")
    target_port = LaunchConfiguration("target_port")
    lateral_sign = LaunchConfiguration("lateral_sign")
    yaw_sign = LaunchConfiguration("yaw_sign")
    desired_angle_deg = LaunchConfiguration("desired_angle_deg")
    forward_axis = LaunchConfiguration("forward_axis")

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("mask_topic", default_value="pipe_cv_segmentation"),
        DeclareLaunchArgument("target_ip", default_value="192.168.2.2"),
        DeclareLaunchArgument("target_port", default_value="14550"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="250"),
        DeclareLaunchArgument("max_sat", default_value="95"),
        DeclareLaunchArgument("min_value", default_value="55"),
        DeclareLaunchArgument("abs_value", default_value="95"),
        DeclareLaunchArgument("local_delta", default_value="5"),
        DeclareLaunchArgument("min_area", default_value="1800"),
        ComposableNodeContainer(
            name="pipe_follow_cv_container",
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
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::CvPipeSegmenterNode",
                    name="cv_pipe_segmenter",
                    parameters=[{
                        "input_topic": "image_raw",
                        "output_topic": mask_topic,
                        "max_sat": LaunchConfiguration("max_sat"),
                        "min_value": LaunchConfiguration("min_value"),
                        "abs_value": LaunchConfiguration("abs_value"),
                        "local_delta": LaunchConfiguration("local_delta"),
                        "min_area": LaunchConfiguration("min_area"),
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::PipeTrackerNode",
                    name="pipe_tracker",
                    parameters=[{
                        "mask_topic": mask_topic,
                        "target_ip": target_ip,
                        "target_port": target_port,
                        "lateral_sign": lateral_sign,
                        "yaw_sign": yaw_sign,
                        "desired_angle_deg": desired_angle_deg,
                        "forward_axis": forward_axis,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        ),
    ])
