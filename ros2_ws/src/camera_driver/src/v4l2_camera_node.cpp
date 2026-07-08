#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace camera_driver
{

class V4L2CameraNode : public rclcpp::Node
{
public:
  explicit V4L2CameraNode(const rclcpp::NodeOptions & options)
  : Node("v4l2_camera", options)
  {
    device_ = declare_parameter<std::string>("device", "/dev/video0");
    width_ = declare_parameter<int>("width", 320);
    height_ = declare_parameter<int>("height", 240);
    fps_ = declare_parameter<int>("fps", 30);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera");
    topic_ = declare_parameter<std::string>("topic", "image_raw");

    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      throw std::runtime_error("failed to open V4L2 device: " + device_);
    }

    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    pub_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::microseconds(1000000 / std::max(1, fps_)),
      std::bind(&V4L2CameraNode::capture, this));
  }

private:
  void capture()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "failed to read frame");
      return;
    }

    if (!frame.isContinuous()) {
      frame = frame.clone();
    }

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = now();
    msg->header.frame_id = frame_id_;
    msg->height = static_cast<uint32_t>(frame.rows);
    msg->width = static_cast<uint32_t>(frame.cols);
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
    msg->data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
    pub_->publish(std::move(msg));
  }

  std::string device_;
  std::string frame_id_;
  std::string topic_;
  int width_;
  int height_;
  int fps_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::V4L2CameraNode)
