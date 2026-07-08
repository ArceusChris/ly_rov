#include <algorithm>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dehaze
{

cv::Mat min_filter(const cv::Mat & image, int radius)
{
  const int kernel_size = radius * 2 + 1;
  cv::Mat eroded;
  cv::erode(image, eroded, cv::getStructuringElement(cv::MORPH_RECT, {kernel_size, kernel_size}));
  return eroded;
}

cv::Mat underwater_dark_channel(const cv::Mat & image, int radius)
{
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  cv::Mat gb_min;
  cv::min(channels[0], channels[1], gb_min);
  return min_filter(gb_min, radius);
}

cv::Vec3f atmospheric_light(const cv::Mat & image, const cv::Mat & dark_channel)
{
  double max_value = 0.0;
  cv::Point max_loc;
  cv::minMaxLoc(dark_channel, nullptr, &max_value, nullptr, &max_loc);
  return image.at<cv::Vec3f>(max_loc);
}

cv::Mat dehaze_udcp(
  const cv::Mat & bgr,
  int radius,
  double omega,
  double min_transmission,
  int blur_size)
{
  cv::Mat image;
  bgr.convertTo(image, CV_32FC3, 1.0 / 255.0);

  const cv::Mat dark = underwater_dark_channel(image, radius);
  const cv::Vec3f air = atmospheric_light(image, dark);
  const cv::Scalar safe_air(
    std::max(air[0], 1e-6f), std::max(air[1], 1e-6f), std::max(air[2], 1e-6f));

  cv::Mat normalized;
  cv::divide(image, safe_air, normalized);
  cv::Mat transmission = 1.0 - omega * underwater_dark_channel(normalized, radius);
  if (blur_size > 1) {
    if (blur_size % 2 == 0) {
      ++blur_size;
    }
    cv::GaussianBlur(transmission, transmission, {blur_size, blur_size}, 0);
  }
  cv::max(transmission, min_transmission, transmission);

  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  for (int i = 0; i < 3; ++i) {
    channels[i] = (channels[i] - air[i]) / transmission + air[i];
  }

  cv::Mat recovered;
  cv::merge(channels, recovered);
  cv::threshold(recovered, recovered, 1.0, 1.0, cv::THRESH_TRUNC);
  cv::threshold(recovered, recovered, 0.0, 0.0, cv::THRESH_TOZERO);
  recovered.convertTo(recovered, CV_8UC3, 255.0);
  return recovered;
}

class UDCPDehazeNode : public rclcpp::Node
{
public:
  explicit UDCPDehazeNode(const rclcpp::NodeOptions & options)
  : Node("udcp_dehaze", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "image_dehazed");
    radius_ = declare_parameter<int>("radius", 5);
    omega_ = declare_parameter<double>("omega", 0.95);
    min_transmission_ = declare_parameter<double>("min_transmission", 0.12);
    blur_size_ = declare_parameter<int>("blur_size", 7);

    pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UDCPDehazeNode::on_image, this, std::placeholders::_1));
  }

private:
  void on_image(sensor_msgs::msg::Image::UniquePtr msg)
  {
    if (msg->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "only bgr8 images are supported");
      return;
    }

    cv::Mat input(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);
    cv::Mat output = dehaze_udcp(input, radius_, omega_, min_transmission_, blur_size_);

    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = msg->header;
    out->height = static_cast<uint32_t>(output.rows);
    out->width = static_cast<uint32_t>(output.cols);
    out->encoding = "bgr8";
    out->is_bigendian = false;
    out->step = static_cast<sensor_msgs::msg::Image::_step_type>(output.cols * output.elemSize());
    out->data.assign(output.data, output.data + output.total() * output.elemSize());
    pub_->publish(std::move(out));
  }

  std::string input_topic_;
  std::string output_topic_;
  int radius_;
  double omega_;
  double min_transmission_;
  int blur_size_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace dehaze

RCLCPP_COMPONENTS_REGISTER_NODE(dehaze::UDCPDehazeNode)

#ifdef UDCP_DEHAZE_SELF_CHECK
#include <cassert>

int main()
{
  cv::Mat input(8, 8, CV_8UC3, cv::Scalar(80, 120, 160));
  cv::Mat output = dehaze::dehaze_udcp(input, 1, 0.95, 0.12, 3);
  assert(output.type() == CV_8UC3);
  assert(output.rows == input.rows);
  assert(output.cols == input.cols);
  return 0;
}
#endif
