import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    os.chdir(get_package_share_directory("dehaze_segmentation"))

    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
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
        DeclareLaunchArgument("target_ip", default_value="192.168.2.2"),
        DeclareLaunchArgument("target_port", default_value="14550"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("desired_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("forward_axis", default_value="250"),
        ComposableNodeContainer(
            name="pipe_follow_container",
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
                    package="dehaze",
                    plugin="dehaze::UDCPDehazeNode",
                    name="udcp_dehaze",
                    parameters=[{
                        "input_topic": "image_raw",
                        "output_topic": "image_dehazed",
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
                        "ros_img_topic_name": "image_dehazed",
                        "config_file": "config/yolov8segworkconfig.json",
                        "msg_pub_topic_name": "hobot_dnn_segmentation",
                        "dump_render_img": 0,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::PipeTrackerNode",
                    name="pipe_tracker",
                    parameters=[{
                        "mask_topic": "hobot_dnn_segmentation",
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
