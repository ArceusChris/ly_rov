from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    raw_image_topic = LaunchConfiguration("raw_image_topic")
    image_topic = LaunchConfiguration("image_topic")
    mask_topic = LaunchConfiguration("mask_topic")
    processed_topic = LaunchConfiguration("processed_topic")

    return LaunchDescription([
        DeclareLaunchArgument("raw_image_topic", default_value="image_raw"),
        DeclareLaunchArgument("image_topic", default_value="image_dehazed"),
        DeclareLaunchArgument("mask_topic", default_value="remote_pipe_cv_segmentation"),
        DeclareLaunchArgument("processed_topic", default_value="remote_processed_image"),
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
            name="remote_processing_preview_cv_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="dehaze",
                    plugin="dehaze::UDCPDehazeNode",
                    name="remote_udcp_dehaze",
                    parameters=[{
                        "input_topic": raw_image_topic,
                        "output_topic": image_topic,
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="rov_pipe_tracker",
                    plugin="rov_pipe_tracker::CvPipeSegmenterNode",
                    name="remote_cv_pipe_segmenter",
                    parameters=[{
                        "input_topic": image_topic,
                        "output_topic": mask_topic,
                        "debug_image_topic": processed_topic,
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
            ],
            output="screen",
        ),
    ])
