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

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="320"),
        DeclareLaunchArgument("height", default_value="240"),
        DeclareLaunchArgument("fps", default_value="30"),
        ComposableNodeContainer(
            name="camera_dehaze_container",
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
            ],
            output="screen",
        ),
    ])
