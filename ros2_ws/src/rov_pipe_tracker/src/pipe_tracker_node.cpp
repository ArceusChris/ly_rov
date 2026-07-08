#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ardupilotmega/mavlink.h>
#include <ai_msgs/msg/perception_targets.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace rov_pipe_tracker
{

struct PipeObservation
{
  bool valid = false;
  double cx = 0.0;
  double cy = 0.0;
  double angle_rad = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;
  size_t pixels = 0;
};

static double wrap_half_pi(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double half_pi = pi * 0.5;
  while (angle > half_pi) {
    angle -= pi;
  }
  while (angle < -half_pi) {
    angle += pi;
  }
  return angle;
}

class PipeTrackerNode : public rclcpp::Node
{
public:
  explicit PipeTrackerNode(const rclcpp::NodeOptions & options)
  : Node("pipe_tracker", options)
  {
    mask_topic_ = declare_parameter<std::string>("mask_topic", "hobot_dnn_segmentation");
    target_ip_ = declare_parameter<std::string>("target_ip", "192.168.2.2");
    target_port_ = declare_parameter<int>("target_port", 14550);
    source_system_ = declare_parameter<int>("source_system", 255);
    source_component_ = declare_parameter<int>("source_component", MAV_COMP_ID_ONBOARD_COMPUTER);
    target_system_ = declare_parameter<int>("target_system", 1);
    mask_label_ = declare_parameter<int>("mask_label", -1);
    min_pixels_ = declare_parameter<int>("min_pixels", 80);
    desired_angle_deg_ = declare_parameter<double>("desired_angle_deg", 90.0);
    forward_axis_ = declare_parameter<int>("forward_axis", 250);
    z_axis_ = declare_parameter<int>("z_axis", 500);
    lateral_gain_ = declare_parameter<double>("lateral_gain", 450.0);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 450.0);
    lateral_sign_ = declare_parameter<double>("lateral_sign", 1.0);
    yaw_sign_ = declare_parameter<double>("yaw_sign", 1.0);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 10.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.5);

    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
      throw std::runtime_error("failed to create UDP socket");
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(static_cast<uint16_t>(target_port_));
    if (::inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) != 1) {
      throw std::runtime_error("invalid target_ip: " + target_ip_);
    }

    sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      mask_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PipeTrackerNode::on_masks, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_))),
      std::bind(&PipeTrackerNode::send_command, this));
  }

  ~PipeTrackerNode() override
  {
    send_manual_control(0, 0, 500, 0);
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

private:
  void on_masks(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    PipeObservation best;
    for (const auto & target : msg->targets) {
      for (const auto & capture : target.captures) {
        auto obs = observe(capture.features, capture.img.width, capture.img.height);
        if (obs.valid && obs.pixels > best.pixels) {
          best = obs;
        }
      }
    }
    last_observation_ = best;
    last_observation_time_ = now();
  }

  PipeObservation observe(const std::vector<uint8_t> & mask, uint32_t width, uint32_t height) const
  {
    PipeObservation obs;
    obs.width = width;
    obs.height = height;
    if (width == 0 || height == 0 || mask.size() < static_cast<size_t>(width) * height) {
      return obs;
    }

    double sx = 0.0;
    double sy = 0.0;
    size_t count = 0;
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const auto v = mask[y * width + x];
        if ((mask_label_ < 0 && v != 0) || v == static_cast<uint8_t>(mask_label_)) {
          sx += x;
          sy += y;
          ++count;
        }
      }
    }
    if (count < static_cast<size_t>(min_pixels_)) {
      return obs;
    }

    obs.cx = sx / count;
    obs.cy = sy / count;
    obs.pixels = count;

    double xx = 0.0;
    double yy = 0.0;
    double xy = 0.0;
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const auto v = mask[y * width + x];
        if ((mask_label_ < 0 && v != 0) || v == static_cast<uint8_t>(mask_label_)) {
          const double dx = x - obs.cx;
          const double dy = y - obs.cy;
          xx += dx * dx;
          yy += dy * dy;
          xy += dx * dy;
        }
      }
    }

    obs.angle_rad = 0.5 * std::atan2(2.0 * xy, xx - yy);
    obs.valid = true;
    return obs;
  }

  void send_command()
  {
    send_heartbeat();

    if (!last_observation_.valid ||
      (now() - last_observation_time_).seconds() > stale_timeout_s_)
    {
      send_manual_control(0, 0, 500, 0);
      return;
    }

    const double x_error = (last_observation_.cx - (last_observation_.width - 1) * 0.5) /
      std::max(1.0, last_observation_.width * 0.5);
    constexpr double pi = 3.14159265358979323846;
    const double desired = desired_angle_deg_ * pi / 180.0;
    const double yaw_error = wrap_half_pi(last_observation_.angle_rad - desired) / (pi * 0.5);

    const int16_t x = clamp_axis(forward_axis_);
    const int16_t y = clamp_axis(std::lround(lateral_sign_ * lateral_gain_ * x_error));
    const int16_t z = static_cast<int16_t>(std::clamp(z_axis_, 0, 1000));
    const int16_t r = clamp_axis(std::lround(yaw_sign_ * yaw_gain_ * yaw_error));
    send_manual_control(x, y, z, r);
  }

  static int16_t clamp_axis(long value)
  {
    return static_cast<int16_t>(std::clamp<long>(value, -1000, 1000));
  }

  void send_heartbeat()
  {
    const auto now_ns = now().nanoseconds();
    if (now_ns - last_heartbeat_ns_ < 1000000000LL) {
      return;
    }
    last_heartbeat_ns_ = now_ns;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
      static_cast<uint8_t>(source_system_),
      static_cast<uint8_t>(source_component_),
      &msg,
      MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,
      0,
      0,
      MAV_STATE_ACTIVE);
    send_mavlink(msg);
  }

  void send_manual_control(int16_t x, int16_t y, int16_t z, int16_t r)
  {
    mavlink_manual_control_t control{};
    control.target = static_cast<uint8_t>(target_system_);
    control.x = x;
    control.y = y;
    control.z = z;
    control.r = r;
    control.buttons = 0;

    mavlink_message_t msg;
    mavlink_msg_manual_control_encode(
      static_cast<uint8_t>(source_system_),
      static_cast<uint8_t>(source_component_),
      &msg,
      &control);
    send_mavlink(msg);
  }

  void send_mavlink(const mavlink_message_t & msg)
  {
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
    ::sendto(
      socket_, buffer.data(), len, 0,
      reinterpret_cast<const sockaddr *>(&target_addr_), sizeof(target_addr_));
  }

  std::string mask_topic_;
  std::string target_ip_;
  int target_port_;
  int source_system_;
  int source_component_;
  int target_system_;
  int mask_label_;
  int min_pixels_;
  double desired_angle_deg_;
  int forward_axis_;
  int z_axis_;
  double lateral_gain_;
  double yaw_gain_;
  double lateral_sign_;
  double yaw_sign_;
  double command_rate_hz_;
  double stale_timeout_s_;

  int socket_ = -1;
  sockaddr_in target_addr_{};
  int64_t last_heartbeat_ns_ = 0;
  PipeObservation last_observation_;
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::PipeTrackerNode)
