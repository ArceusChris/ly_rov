import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share_dir = get_package_share_directory("dehaze_segmentation")
    os.chdir(share_dir)

    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    config_file = LaunchConfiguration("config_file")
    segmentation_topic = LaunchConfiguration("segmentation_topic")

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("config_file", default_value="config/yolov8segworkconfig.json"),
        DeclareLaunchArgument("segmentation_topic", default_value="hobot_dnn_segmentation"),
        ComposableNodeContainer(
            name="dehaze_segmentation_container",
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
                        "config_file": config_file,
                        "msg_pub_topic_name": segmentation_topic,
                        "dump_render_img": 0,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        ),
    ])
