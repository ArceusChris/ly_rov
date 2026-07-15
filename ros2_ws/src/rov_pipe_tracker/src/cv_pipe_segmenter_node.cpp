#include <algorithm>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ai_msgs/msg/perception_targets.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rov_pipe_tracker
{

class CvPipeSegmenterNode : public rclcpp::Node
{
public:
  explicit CvPipeSegmenterNode(const rclcpp::NodeOptions & options)
  : Node("cv_pipe_segmenter", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "pipe_cv_segmentation");
    min_area_ = declare_parameter<int>("min_area", 1800);
    max_sat_ = declare_parameter<int>("max_sat", 95);
    min_value_ = declare_parameter<int>("min_value", 55);
    abs_value_ = declare_parameter<int>("abs_value", 95);
    local_delta_ = declare_parameter<int>("local_delta", 5);
    min_height_ratio_ = declare_parameter<double>("min_height_ratio", 0.18);
    min_aspect_ = declare_parameter<double>("min_aspect", 1.0);

    pub_ = create_publisher<ai_msgs::msg::PerceptionTargets>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CvPipeSegmenterNode::on_image, this, std::placeholders::_1));
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat frame;
    if (!to_bgr(*msg, frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "unsupported image encoding: %s",
        msg->encoding.c_str());
      return;
    }

    const auto mask = segment(frame);
    auto out = std::make_unique<ai_msgs::msg::PerceptionTargets>();
    out->header = msg->header;

    ai_msgs::msg::Capture capture;
    capture.img.height = static_cast<uint32_t>(mask.rows);
    capture.img.width = static_cast<uint32_t>(mask.cols);
    capture.img.step = 1;
    if (mask.isContinuous()) {
      capture.features.assign(mask.data, mask.data + mask.total());
    } else {
      capture.features.reserve(mask.total());
      for (int y = 0; y < mask.rows; ++y) {
        const auto * row = mask.ptr<uint8_t>(y);
        capture.features.insert(capture.features.end(), row, row + mask.cols);
      }
    }

    ai_msgs::msg::Target target;
    target.set__type("pipe");
    target.captures.emplace_back(std::move(capture));
    out->targets.emplace_back(std::move(target));
    pub_->publish(std::move(out));
  }

  static bool to_bgr(const sensor_msgs::msg::Image & msg, cv::Mat & frame)
  {
    if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
      return false;
    }

    if (msg.encoding == "bgr8") {
      frame = cv::Mat(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
        const_cast<uint8_t *>(msg.data.data()), msg.step);
      if (!frame.isContinuous()) {
        frame = frame.clone();
      }
      return true;
    }

    if (msg.encoding == "rgb8") {
      cv::Mat rgb(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
        const_cast<uint8_t *>(msg.data.data()), msg.step);
      cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
      return true;
    }

    return false;
  }

  cv::Mat segment(const cv::Mat & frame) const
  {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> ch;
    cv::split(hsv, ch);
    cv::Mat value_i;
    ch[2].convertTo(value_i, CV_16S);

    cv::Mat local_u8;
    cv::blur(ch[2], local_u8, cv::Size(51, 51));
    cv::Mat local_i;
    local_u8.convertTo(local_i, CV_16S);
    local_i += local_delta_;

    cv::Mat low_sat;
    cv::Mat min_value;
    cv::Mat local_bright;
    cv::Mat abs_bright;
    cv::compare(ch[1], max_sat_, low_sat, cv::CMP_LT);
    cv::compare(value_i, min_value_, min_value, cv::CMP_GT);
    cv::compare(value_i, local_i, local_bright, cv::CMP_GT);
    cv::compare(value_i, abs_value_, abs_bright, cv::CMP_GT);

    cv::Mat bright;
    cv::bitwise_or(local_bright, abs_bright, bright);
    cv::Mat mask;
    cv::bitwise_and(low_sat, min_value, mask);
    cv::bitwise_and(mask, bright, mask);

    // ponytail: tuned for pipe2.mp4; CLI/ROS params are enough until lighting changes a lot.
    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 25)));
    cv::morphologyEx(
      mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 9)));

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);

    int best_label = 0;
    double best_score = 0.0;
    for (int label = 1; label < count; ++label) {
      const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
      const int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (area < min_area_ || h < frame.rows * min_height_ratio_ || w < 8) {
        continue;
      }
      const double aspect = static_cast<double>(h) / std::max(1, w);
      if (aspect < min_aspect_) {
        continue;
      }
      const double center = x + w * 0.5;
      const double center_bias = 1.0 - std::abs(center - frame.cols * 0.5) / std::max(1.0, frame.cols * 0.5);
      const double score =
        area * (0.8 + std::min(aspect, 5.0) * 0.2) * (0.7 + std::max(0.0, center_bias) * 0.3);
      if (score > best_score) {
        best_score = score;
        best_label = label;
      }
    }

    cv::Mat best = cv::Mat::zeros(mask.size(), CV_8UC1);
    if (best_label != 0) {
      cv::compare(labels, cv::Scalar(best_label), best, cv::CMP_EQ);
      best /= 255;
    }
    return best;
  }

  std::string input_topic_;
  std::string output_topic_;
  int min_area_;
  int max_sat_;
  int min_value_;
  int abs_value_;
  int local_delta_;
  double min_height_ratio_;
  double min_aspect_;

  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::CvPipeSegmenterNode)
