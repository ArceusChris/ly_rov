#include <algorithm>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ai_msgs/msg/perception_targets.hpp>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "");
    min_area_ = declare_parameter<int>("min_area", 1800);
    min_red_ = declare_parameter<int>("min_red", 150);
    min_green_ = declare_parameter<int>("min_green", 150);
    min_blue_ = declare_parameter<int>("min_blue", 150);
    max_channel_diff_ = declare_parameter<int>("max_channel_diff", 45);
    max_red_blue_diff_ = declare_parameter<int>("max_red_blue_diff", 55);
    min_height_ratio_ = declare_parameter<double>("min_height_ratio", 0.18);
    min_aspect_ = declare_parameter<double>("min_aspect", 1.0);
    max_aspect_ = declare_parameter<double>("max_aspect", 8.0);
    erode_kernel_size_ = declare_parameter<int>("erode_kernel_size", 10);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&CvPipeSegmenterNode::on_set_parameters, this, std::placeholders::_1));

    pub_ = create_publisher<ai_msgs::msg::PerceptionTargets>(output_topic_, rclcpp::SensorDataQoS());
    if (!debug_image_topic_.empty()) {
      debug_image_pub_ =
        create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());
    }
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CvPipeSegmenterNode::on_image, this, std::placeholders::_1));
  }

private:
  struct Parameters
  {
    int min_area = 1800;
    int min_red = 150;
    int min_green = 150;
    int min_blue = 150;
    int max_channel_diff = 45;
    int max_red_blue_diff = 55;
    double min_height_ratio = 0.18;
    double min_aspect = 1.0;
    double max_aspect = 8.0;
    int erode_kernel_size = 10;
  };

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    Parameters next = parameters_snapshot();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      try {
        if (name == "min_area") {
          next.min_area = parameter.as_int();
        } else if (name == "min_red") {
          next.min_red = parameter.as_int();
        } else if (name == "min_green") {
          next.min_green = parameter.as_int();
        } else if (name == "min_blue") {
          next.min_blue = parameter.as_int();
        } else if (name == "max_channel_diff") {
          next.max_channel_diff = parameter.as_int();
        } else if (name == "max_red_blue_diff") {
          next.max_red_blue_diff = parameter.as_int();
        } else if (name == "min_height_ratio") {
          next.min_height_ratio = parameter.as_double();
        } else if (name == "min_aspect") {
          next.min_aspect = parameter.as_double();
        } else if (name == "max_aspect") {
          next.max_aspect = parameter.as_double();
        } else if (name == "erode_kernel_size") {
          next.erode_kernel_size = parameter.as_int();
        }
      } catch (const rclcpp::ParameterTypeException & error) {
        result.successful = false;
        result.reason = name + ": " + error.what();
        return result;
      }
    }

    if (next.min_area < 0 || next.min_red < 0 || next.min_red > 255 ||
      next.min_green < 0 || next.min_green > 255 || next.min_blue < 0 || next.min_blue > 255 ||
      next.max_channel_diff < 0 || next.max_channel_diff > 255 ||
      next.max_red_blue_diff < -255 || next.max_red_blue_diff > 255 ||
      next.min_height_ratio < 0.0 || next.min_aspect < 0.0 || next.max_aspect < 0.0 ||
      next.erode_kernel_size < 1)
    {
      result.successful = false;
      result.reason = "parameter out of range";
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(parameter_mutex_);
      min_area_ = next.min_area;
      min_red_ = next.min_red;
      min_green_ = next.min_green;
      min_blue_ = next.min_blue;
      max_channel_diff_ = next.max_channel_diff;
      max_red_blue_diff_ = next.max_red_blue_diff;
      min_height_ratio_ = next.min_height_ratio;
      min_aspect_ = next.min_aspect;
      max_aspect_ = next.max_aspect;
      erode_kernel_size_ = next.erode_kernel_size;
    }
    return result;
  }

  Parameters parameters_snapshot() const
  {
    std::lock_guard<std::mutex> lock(parameter_mutex_);
    return Parameters{
      min_area_,
      min_red_,
      min_green_,
      min_blue_,
      max_channel_diff_,
      max_red_blue_diff_,
      min_height_ratio_,
      min_aspect_,
      max_aspect_,
      erode_kernel_size_};
  }

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
    publish_debug_image(*msg, frame, mask);
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

  void publish_debug_image(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & frame,
    const cv::Mat & mask)
  {
    if (!debug_image_pub_) {
      return;
    }

    cv::Mat binary;
    cv::compare(mask, 0, binary, cv::CMP_GT);

    cv::Mat tint = frame.clone();
    tint.setTo(cv::Scalar(0, 255, 80), binary);

    cv::Mat overlay;
    cv::addWeighted(frame, 0.68, tint, 0.32, 0.0, overlay);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(overlay, contours, -1, cv::Scalar(0, 0, 255), 2);
    if (!overlay.isContinuous()) {
      overlay = overlay.clone();
    }

    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = source.header;
    out->height = static_cast<uint32_t>(overlay.rows);
    out->width = static_cast<uint32_t>(overlay.cols);
    out->encoding = "bgr8";
    out->is_bigendian = false;
    out->step = static_cast<sensor_msgs::msg::Image::_step_type>(overlay.cols * overlay.elemSize());
    out->data.assign(overlay.data, overlay.data + overlay.total() * overlay.elemSize());
    debug_image_pub_->publish(std::move(out));
  }

  cv::Mat segment(const cv::Mat & frame)
  {
    const auto params = parameters_snapshot();

    std::vector<cv::Mat> ch;
    cv::split(frame, ch);
    const auto & blue = ch[0];
    const auto & green = ch[1];
    const auto & red = ch[2];

    double blue_min = 0.0;
    double blue_max = 0.0;
    double green_min = 0.0;
    double green_max = 0.0;
    double red_min = 0.0;
    double red_max = 0.0;
    cv::minMaxLoc(blue, &blue_min, &blue_max);
    cv::minMaxLoc(green, &green_min, &green_max);
    cv::minMaxLoc(red, &red_min, &red_max);

    cv::Mat red_ok;
    cv::Mat green_ok;
    cv::Mat blue_ok;
    cv::compare(red, params.min_red, red_ok, cv::CMP_GT);
    cv::compare(green, params.min_green, green_ok, cv::CMP_GT);
    cv::compare(blue, params.min_blue, blue_ok, cv::CMP_GT);

    cv::Mat max_channel;
    cv::Mat min_channel;
    cv::max(red, green, max_channel);
    cv::max(max_channel, blue, max_channel);
    cv::min(red, green, min_channel);
    cv::min(min_channel, blue, min_channel);

    cv::Mat channel_diff;
    cv::subtract(max_channel, min_channel, channel_diff);
    cv::Mat neutral_ok;
    cv::compare(channel_diff, params.max_channel_diff, neutral_ok, cv::CMP_LE);

    cv::Mat red_i;
    cv::Mat blue_i;
    red.convertTo(red_i, CV_16S);
    blue.convertTo(blue_i, CV_16S);
    cv::Mat red_blue_diff;
    cv::subtract(red_i, blue_i, red_blue_diff);
    cv::Mat not_wood;
    cv::compare(red_blue_diff, params.max_red_blue_diff, not_wood, cv::CMP_LE);

    cv::Mat mask;
    cv::bitwise_and(red_ok, green_ok, mask);
    cv::bitwise_and(mask, blue_ok, mask);
    cv::bitwise_and(mask, neutral_ok, mask);
    cv::bitwise_and(mask, not_wood, mask);
    const int threshold_pixels = cv::countNonZero(mask);

    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 25)));
    cv::morphologyEx(
      mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 9)));
    const int morph_pixels = cv::countNonZero(mask);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);

    int best_label = 0;
    double best_score = 0.0;
    int rejected_area = 0;
    int rejected_min_aspect = 0;
    int rejected_max_aspect = 0;
    int largest_x = 0;
    int largest_y = 0;
    int largest_w = 0;
    int largest_h = 0;
    int largest_area = 0;
    double largest_aspect = 0.0;
    for (int label = 1; label < count; ++label) {
      const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
      const int y = stats.at<int>(label, cv::CC_STAT_TOP);
      const int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      const double aspect = static_cast<double>(h) / std::max(1, w);
      if (area > largest_area) {
        largest_x = x;
        largest_y = y;
        largest_w = w;
        largest_h = h;
        largest_area = area;
        largest_aspect = aspect;
      }
      if (area < params.min_area || h < frame.rows * params.min_height_ratio || w < 8) {
        ++rejected_area;
        continue;
      }
      if (aspect < params.min_aspect) {
        ++rejected_min_aspect;
        continue;
      }
      if (params.max_aspect > 0.0 && aspect > params.max_aspect) {
        ++rejected_max_aspect;
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
    const int selected_pixels = cv::countNonZero(best);
    if (params.erode_kernel_size > 1 && selected_pixels > 0) {
      const auto kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(params.erode_kernel_size, params.erode_kernel_size));
      cv::erode(best, best, kernel);
    }
    const int output_pixels = cv::countNonZero(best);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cv rgb segment stats: frame=%dx%d r=[%.0f,%.0f] g=[%.0f,%.0f] b=[%.0f,%.0f] "
      "threshold_px=%d morph_px=%d components=%d rejected_area=%d rejected_min_aspect=%d "
      "rejected_max_aspect=%d "
      "largest=(x=%d y=%d w=%d h=%d area=%d aspect=%.2f) best_label=%d selected_px=%d output_px=%d "
      "params(min_red=%d min_green=%d min_blue=%d max_channel_diff=%d max_red_blue_diff=%d "
      "min_area=%d min_height_ratio=%.2f min_aspect=%.2f max_aspect=%.2f erode_kernel_size=%d)",
      frame.cols, frame.rows, red_min, red_max, green_min, green_max, blue_min, blue_max,
      threshold_pixels, morph_pixels, std::max(0, count - 1), rejected_area, rejected_min_aspect,
      rejected_max_aspect,
      largest_x, largest_y, largest_w, largest_h, largest_area, largest_aspect,
      best_label, selected_pixels, output_pixels, params.min_red, params.min_green, params.min_blue,
      params.max_channel_diff, params.max_red_blue_diff, params.min_area,
      params.min_height_ratio, params.min_aspect, params.max_aspect, params.erode_kernel_size);
    return best;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string debug_image_topic_;
  int min_area_;
  int min_red_;
  int min_green_;
  int min_blue_;
  int max_channel_diff_;
  int max_red_blue_diff_;
  double min_height_ratio_;
  double min_aspect_;
  double max_aspect_;
  int erode_kernel_size_;
  mutable std::mutex parameter_mutex_;

  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::CvPipeSegmenterNode)
