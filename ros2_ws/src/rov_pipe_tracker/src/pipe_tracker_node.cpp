#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ardupilotmega/mavlink.h>
#include <ai_msgs/msg/perception_targets.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    manual_command_topic_ =
      declare_parameter<std::string>("manual_command_topic", "manual_control_command");
    command_log_topic_ = declare_parameter<std::string>("command_log_topic", "pipe_tracker_command_log");
    output_command_topic_ = declare_parameter<std::string>("output_command_topic", "");
    mavlink_enabled_ = declare_parameter<bool>("mavlink_enabled", true);
    target_ip_ = declare_parameter<std::string>("target_ip", "192.168.2.2");
    target_port_ = declare_parameter<int>("target_port", 14550);
    source_system_ = declare_parameter<int>("source_system", 255);
    source_component_ = declare_parameter<int>("source_component", MAV_COMP_ID_ONBOARD_COMPUTER);
    target_system_ = declare_parameter<int>("target_system", 1);
    target_component_ = declare_parameter<int>("target_component", MAV_COMP_ID_AUTOPILOT1);
    udp_bind_port_ = declare_parameter<int>("udp_bind_port", 0);
    learn_target_from_udp_ = declare_parameter<bool>("learn_target_from_udp", false);
    set_manual_mode_ = declare_parameter<bool>("set_manual_mode", true);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    manual_mode_ = declare_parameter<int>("manual_mode", 19);
    startup_command_retries_ = declare_parameter<int>("startup_command_retries", 5);
    startup_command_interval_s_ = declare_parameter<double>("startup_command_interval_s", 0.5);
    manual_control_enabled_ = declare_parameter<bool>("manual_control_enabled", true);
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
    manual_override_timeout_s_ = declare_parameter<double>("manual_override_timeout_s", 0.5);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&PipeTrackerNode::on_set_parameters, this, std::placeholders::_1));

    if (!output_command_topic_.empty()) {
      output_command_pub_ =
        create_publisher<std_msgs::msg::Int16MultiArray>(output_command_topic_, rclcpp::QoS(10));
    }

    if (mavlink_enabled_) {
      socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (socket_ < 0) {
        throw std::runtime_error("failed to create UDP socket");
      }
      int reuse = 1;
      ::setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

      if (udp_bind_port_ > 0) {
        sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind_addr.sin_port = htons(static_cast<uint16_t>(udp_bind_port_));
        if (::bind(socket_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) != 0) {
          const std::string error = std::strerror(errno);
          ::close(socket_);
          socket_ = -1;
          throw std::runtime_error("failed to bind UDP port " + std::to_string(udp_bind_port_) +
            ": " + error);
        }
        const int flags = ::fcntl(socket_, F_GETFL, 0);
        if (flags >= 0) {
          ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
        }
      }

      std::memset(&target_addr_, 0, sizeof(target_addr_));
      target_addr_.sin_family = AF_INET;
      target_addr_.sin_port = htons(static_cast<uint16_t>(target_port_));
      if (::inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) != 1) {
        throw std::runtime_error("invalid target_ip: " + target_ip_);
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "MAVLink enabled=%d target fallback=%s:%d bind_port=%d learn_target_from_udp=%d "
      "mask_topic=%s output_command_topic=%s set_manual_mode=%d auto_arm=%d",
      mavlink_enabled_,
      target_ip_.c_str(), target_port_, udp_bind_port_, learn_target_from_udp_,
      mask_topic_.c_str(), output_command_topic_.c_str(), set_manual_mode_, auto_arm_);

    sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      mask_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PipeTrackerNode::on_masks, this, std::placeholders::_1));
    manual_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      manual_command_topic_, rclcpp::QoS(10),
      std::bind(&PipeTrackerNode::on_manual_command, this, std::placeholders::_1));
    command_log_pub_ = create_publisher<std_msgs::msg::String>(command_log_topic_, rclcpp::QoS(50));
    arm_service_ = create_service<std_srvs::srv::SetBool>(
      "~/arm",
      std::bind(
        &PipeTrackerNode::on_arm_service, this, std::placeholders::_1,
        std::placeholders::_2));
    manual_mode_service_ = create_service<std_srvs::srv::Trigger>(
      "~/manual_mode",
      std::bind(
        &PipeTrackerNode::on_manual_mode_service, this, std::placeholders::_1,
        std::placeholders::_2));
    manual_control_service_ = create_service<std_srvs::srv::SetBool>(
      "~/manual_control_enabled",
      std::bind(
        &PipeTrackerNode::on_manual_control_service, this, std::placeholders::_1,
        std::placeholders::_2));
    if (mavlink_enabled_ && learn_target_from_udp_) {
      receive_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&PipeTrackerNode::receive_mavlink, this));
    }

    reset_command_timer();
  }

  ~PipeTrackerNode() override
  {
    send_manual_control(0, 0, 500, 0);
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

private:
  struct ControlParameters
  {
    bool manual_control_enabled = true;
    int mask_label = -1;
    int min_pixels = 80;
    double desired_angle_deg = 90.0;
    int forward_axis = 250;
    int z_axis = 500;
    double lateral_gain = 450.0;
    double yaw_gain = 450.0;
    double lateral_sign = 1.0;
    double yaw_sign = 1.0;
    double command_rate_hz = 10.0;
    double stale_timeout_s = 0.5;
    double manual_override_timeout_s = 0.5;
  };

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    ControlParameters next = control_parameters_snapshot();
    bool command_rate_changed = false;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      try {
        if (name == "manual_control_enabled") {
          next.manual_control_enabled = parameter.as_bool();
        } else if (name == "mask_label") {
          next.mask_label = parameter.as_int();
        } else if (name == "min_pixels") {
          next.min_pixels = parameter.as_int();
        } else if (name == "desired_angle_deg") {
          next.desired_angle_deg = parameter.as_double();
        } else if (name == "forward_axis") {
          next.forward_axis = parameter.as_int();
        } else if (name == "z_axis") {
          next.z_axis = parameter.as_int();
        } else if (name == "lateral_gain") {
          next.lateral_gain = parameter.as_double();
        } else if (name == "yaw_gain") {
          next.yaw_gain = parameter.as_double();
        } else if (name == "lateral_sign") {
          next.lateral_sign = parameter.as_double();
        } else if (name == "yaw_sign") {
          next.yaw_sign = parameter.as_double();
        } else if (name == "command_rate_hz") {
          next.command_rate_hz = parameter.as_double();
          command_rate_changed = true;
        } else if (name == "stale_timeout_s") {
          next.stale_timeout_s = parameter.as_double();
        } else if (name == "manual_override_timeout_s") {
          next.manual_override_timeout_s = parameter.as_double();
        }
      } catch (const rclcpp::ParameterTypeException & error) {
        result.successful = false;
        result.reason = name + ": " + error.what();
        return result;
      }
    }

    if (next.min_pixels < 0 || next.forward_axis < -1000 || next.forward_axis > 1000 ||
      next.z_axis < 0 || next.z_axis > 1000 || next.lateral_gain < 0.0 ||
      next.yaw_gain < 0.0 || next.command_rate_hz < 1.0 || next.stale_timeout_s < 0.0 ||
      next.manual_override_timeout_s < 0.0)
    {
      result.successful = false;
      result.reason = "parameter out of range";
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(parameter_mutex_);
      manual_control_enabled_ = next.manual_control_enabled;
      mask_label_ = next.mask_label;
      min_pixels_ = next.min_pixels;
      desired_angle_deg_ = next.desired_angle_deg;
      forward_axis_ = next.forward_axis;
      z_axis_ = next.z_axis;
      lateral_gain_ = next.lateral_gain;
      yaw_gain_ = next.yaw_gain;
      lateral_sign_ = next.lateral_sign;
      yaw_sign_ = next.yaw_sign;
      command_rate_hz_ = next.command_rate_hz;
      stale_timeout_s_ = next.stale_timeout_s;
      manual_override_timeout_s_ = next.manual_override_timeout_s;
    }

    if (!next.manual_control_enabled) {
      send_manual_control(0, 0, 500, 0);
    }
    if (command_rate_changed) {
      reset_command_timer();
    }
    return result;
  }

  ControlParameters control_parameters_snapshot() const
  {
    std::lock_guard<std::mutex> lock(parameter_mutex_);
    return ControlParameters{
      manual_control_enabled_,
      mask_label_,
      min_pixels_,
      desired_angle_deg_,
      forward_axis_,
      z_axis_,
      lateral_gain_,
      yaw_gain_,
      lateral_sign_,
      yaw_sign_,
      command_rate_hz_,
      stale_timeout_s_,
      manual_override_timeout_s_};
  }

  void reset_command_timer()
  {
    const auto params = control_parameters_snapshot();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, params.command_rate_hz))),
      std::bind(&PipeTrackerNode::send_command, this));
  }

  void on_masks(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    const auto params = control_parameters_snapshot();
    PipeObservation best;
    for (const auto & target : msg->targets) {
      for (const auto & capture : target.captures) {
        auto obs = observe(
          capture.features, capture.img.width, capture.img.height, params.mask_label,
          params.min_pixels);
        if (obs.valid && obs.pixels > best.pixels) {
          best = obs;
        }
      }
    }
    last_observation_ = best;
    last_observation_time_ = now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "pipe observation: valid=%d pixels=%zu center=(%.1f, %.1f) angle_deg=%.1f size=%ux%u",
      best.valid, best.pixels, best.cx, best.cy, best.angle_rad * 180.0 / 3.14159265358979323846,
      best.width, best.height);
  }

  void on_manual_command(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "manual command requires [x, y, z, r, override_enabled]");
      return;
    }

    manual_x_ = clamp_axis(msg->data[0]);
    manual_y_ = clamp_axis(msg->data[1]);
    manual_z_ = static_cast<int16_t>(std::clamp<int>(msg->data[2], 0, 1000));
    manual_r_ = clamp_axis(msg->data[3]);
    manual_override_enabled_ = msg->data[4] != 0;
    last_manual_command_time_ = now();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "manual override: enabled=%d x=%d y=%d z=%d r=%d",
      manual_override_enabled_, manual_x_, manual_y_, manual_z_, manual_r_);
  }

  PipeObservation observe(
    const std::vector<float> & mask, uint32_t width, uint32_t height, int mask_label,
    int min_pixels) const
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
        if ((mask_label < 0 && v != 0.0f) || v == static_cast<float>(mask_label)) {
          sx += x;
          sy += y;
          ++count;
        }
      }
    }
    if (count < static_cast<size_t>(min_pixels)) {
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
        if ((mask_label < 0 && v != 0.0f) || v == static_cast<float>(mask_label)) {
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

  void publish_command_log(
    const std::string & mode,
    int16_t x,
    int16_t y,
    int16_t z,
    int16_t r,
    const std::string & reason = "",
    double x_error = std::numeric_limits<double>::quiet_NaN(),
    double yaw_error = std::numeric_limits<double>::quiet_NaN())
  {
    if (!command_log_pub_) {
      return;
    }

    std::ostringstream line;
    line << std::fixed << std::setprecision(2)
         << "t=" << now().seconds()
         << " mode=" << mode
         << " x=" << x
         << " y=" << y
         << " z=" << z
         << " r=" << r;
    if (!reason.empty()) {
      line << " reason=" << reason;
    }
    if (std::isfinite(x_error)) {
      line << std::setprecision(3) << " x_error=" << x_error;
    }
    if (std::isfinite(yaw_error)) {
      line << std::setprecision(3) << " yaw_error=" << yaw_error;
    }

    std_msgs::msg::String msg;
    msg.data = line.str();
    command_log_pub_->publish(msg);
  }

  void send_command()
  {
    const auto params = control_parameters_snapshot();
    if (mavlink_enabled_) {
      send_heartbeat();
      send_startup_commands();
    }

    if (!params.manual_control_enabled) {
      publish_command_log("disabled", 0, 0, 500, 0, "manual_control_stream_off");
      return;
    }

    if (mavlink_enabled_ &&
      (!vehicle_armed_ || vehicle_custom_mode_ != static_cast<uint32_t>(manual_mode_)))
    {
      send_manual_control(0, 0, 500, 0);
      publish_command_log("neutral", 0, 0, 500, 0, "not_armed_or_not_manual");
      return;
    }

    if (manual_override_enabled_) {
      if ((now() - last_manual_command_time_).seconds() > params.manual_override_timeout_s) {
        send_manual_control(0, 0, 500, 0);
        publish_command_log("manual_override", 0, 0, 500, 0, "manual_command_stale");
        return;
      }
      send_manual_control(manual_x_, manual_y_, manual_z_, manual_r_);
      publish_command_log("manual_override", manual_x_, manual_y_, manual_z_, manual_r_);
      return;
    }

    if (!last_observation_.valid ||
      (now() - last_observation_time_).seconds() > params.stale_timeout_s)
    {
      send_manual_control(0, 0, 500, 0);
      publish_command_log("neutral", 0, 0, 500, 0, "mask_lost_or_stale");
      return;
    }

    const double x_error = (last_observation_.cx - (last_observation_.width - 1) * 0.5) /
      std::max(1.0, last_observation_.width * 0.5);
    constexpr double pi = 3.14159265358979323846;
    const double desired = params.desired_angle_deg * pi / 180.0;
    const double yaw_error = wrap_half_pi(last_observation_.angle_rad - desired) / (pi * 0.5);

    const int16_t x = clamp_axis(params.forward_axis);
    const int16_t y = clamp_axis(std::lround(params.lateral_sign * params.lateral_gain * x_error));
    const int16_t z = static_cast<int16_t>(std::clamp(params.z_axis, 0, 1000));
    const int16_t r = clamp_axis(std::lround(params.yaw_sign * params.yaw_gain * yaw_error));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "manual_control: x=%d y=%d z=%d r=%d target=%s:%d x_error=%.3f yaw_error=%.3f",
      x, y, z, r, target_ip_.c_str(), target_port_, x_error, yaw_error);
    send_manual_control(x, y, z, r);
    publish_command_log("auto", x, y, z, r, "", x_error, yaw_error);
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

  void send_startup_commands()
  {
    if (!set_manual_mode_ && !auto_arm_) {
      return;
    }
    if (startup_command_attempts_ >= std::max(1, startup_command_retries_)) {
      return;
    }

    const auto now_ns = now().nanoseconds();
    const auto interval_ns = static_cast<int64_t>(
      std::max(0.1, startup_command_interval_s_) * 1000000000.0);
    if (last_startup_command_ns_ != 0 && now_ns - last_startup_command_ns_ < interval_ns) {
      return;
    }
    last_startup_command_ns_ = now_ns;
    ++startup_command_attempts_;

    if (set_manual_mode_) {
      send_set_mode(static_cast<uint32_t>(manual_mode_));
    }
    if (auto_arm_) {
      send_arm_command(true);
    }
    RCLCPP_INFO(
      get_logger(), "startup MAVLink command attempt %d/%d: manual_mode=%d auto_arm=%d",
      startup_command_attempts_, startup_command_retries_, set_manual_mode_, auto_arm_);
  }

  void send_set_mode(uint32_t custom_mode)
  {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(source_system_),
      static_cast<uint8_t>(source_component_),
      &msg,
      static_cast<uint8_t>(target_system_),
      static_cast<uint8_t>(target_component_),
      MAV_CMD_DO_SET_MODE,
      0,
      static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
      static_cast<float>(custom_mode),
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f);
    send_mavlink(msg);
  }

  void send_arm_command(bool arm)
  {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
      static_cast<uint8_t>(source_system_),
      static_cast<uint8_t>(source_component_),
      &msg,
      static_cast<uint8_t>(target_system_),
      static_cast<uint8_t>(target_component_),
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      arm ? 1.0f : 0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f,
      0.0f);
    send_mavlink(msg);
  }

  void send_manual_control(int16_t x, int16_t y, int16_t z, int16_t r)
  {
    if (output_command_pub_) {
      std_msgs::msg::Int16MultiArray out;
      out.data = {x, y, z, r};
      output_command_pub_->publish(out);
    }
    if (!mavlink_enabled_) {
      return;
    }

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

  void receive_mavlink()
  {
    if (socket_ < 0) {
      return;
    }

    std::array<uint8_t, 2048> buffer{};
    while (true) {
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);
      const auto received = ::recvfrom(
        socket_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender), &sender_len);
      if (received < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "recvfrom failed: %s", std::strerror(errno));
        }
        return;
      }
      if (received == 0) {
        return;
      }

      for (ssize_t i = 0; i < received; ++i) {
        mavlink_message_t msg;
        if (!mavlink_parse_char(MAVLINK_COMM_0, buffer[static_cast<size_t>(i)], &msg, &parse_status_)) {
          continue;
        }
        if (msg.sysid != static_cast<uint8_t>(target_system_)) {
          continue;
        }
        learn_target(sender);
        log_mavlink_feedback(msg);
      }
    }
  }

  void learn_target(const sockaddr_in & sender)
  {
    if (have_learned_target_ &&
      learned_target_addr_.sin_addr.s_addr == sender.sin_addr.s_addr &&
      learned_target_addr_.sin_port == sender.sin_port)
    {
      return;
    }

    learned_target_addr_ = sender;
    have_learned_target_ = true;
    RCLCPP_INFO(
      get_logger(), "learned MAVLink target from UDP sysid=%d: %s",
      target_system_, format_addr(learned_target_addr_).c_str());
  }

  void log_mavlink_feedback(const mavlink_message_t & msg)
  {
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(&msg, &heartbeat);
      const bool armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      vehicle_armed_ = armed;
      vehicle_custom_mode_ = heartbeat.custom_mode;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ArduSub heartbeat: armed=%d base_mode=0x%02x custom_mode=%u system_status=%u",
        armed, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
      return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      mavlink_command_ack_t ack;
      mavlink_msg_command_ack_decode(&msg, &ack);
      RCLCPP_INFO(
        get_logger(), "MAVLink command ack: command=%u result=%u progress=%u result_param2=%d",
        ack.command, ack.result, ack.progress, ack.result_param2);
      return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
      mavlink_statustext_t status_text;
      mavlink_msg_statustext_decode(&msg, &status_text);
      std::array<char, sizeof(status_text.text) + 1> text{};
      std::memcpy(text.data(), status_text.text, sizeof(status_text.text));
      RCLCPP_WARN(
        get_logger(), "ArduSub status text: severity=%u text=%s",
        status_text.severity, text.data());
    }
  }

  void send_mavlink(const mavlink_message_t & msg)
  {
    if (socket_ < 0) {
      return;
    }

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
    const auto & destination = have_learned_target_ ? learned_target_addr_ : target_addr_;
    const auto sent = ::sendto(
      socket_, buffer.data(), len, 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
    if (sent < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "sendto failed: %s", std::strerror(errno));
    }
  }

  static std::string format_addr(const sockaddr_in & addr)
  {
    char ip[INET_ADDRSTRLEN] = {};
    if (::inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip)) == nullptr) {
      return "unknown";
    }
    return std::string(ip) + ":" + std::to_string(ntohs(addr.sin_port));
  }

  void on_arm_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    send_arm_command(request->data);
    response->success = true;
    response->message = request->data ? "arm command sent" : "disarm command sent";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void on_manual_mode_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_set_mode(static_cast<uint32_t>(manual_mode_));
    response->success = true;
    response->message = "manual mode command sent";
    RCLCPP_INFO(get_logger(), "%s: mode=%d", response->message.c_str(), manual_mode_);
  }

  void on_manual_control_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const auto result =
      set_parameter(rclcpp::Parameter("manual_control_enabled", request->data));
    if (!result.successful) {
      response->success = false;
      response->message = result.reason;
      return;
    }
    if (!request->data) {
      send_manual_control(0, 0, 500, 0);
    }
    response->success = true;
    response->message = request->data ?
      "manual control stream enabled" : "manual control stream disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  std::string mask_topic_;
  std::string manual_command_topic_;
  std::string command_log_topic_;
  std::string output_command_topic_;
  bool mavlink_enabled_;
  std::string target_ip_;
  int target_port_;
  int source_system_;
  int source_component_;
  int target_system_;
  int target_component_;
  int udp_bind_port_;
  bool learn_target_from_udp_;
  bool set_manual_mode_;
  bool auto_arm_;
  int manual_mode_;
  int startup_command_retries_;
  double startup_command_interval_s_;
  bool manual_control_enabled_;
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
  double manual_override_timeout_s_;
  mutable std::mutex parameter_mutex_;

  int socket_ = -1;
  sockaddr_in target_addr_{};
  sockaddr_in learned_target_addr_{};
  bool have_learned_target_ = false;
  mavlink_status_t parse_status_{};
  bool vehicle_armed_ = false;
  uint32_t vehicle_custom_mode_ = 0;
  int64_t last_heartbeat_ns_ = 0;
  int64_t last_startup_command_ns_ = 0;
  int startup_command_attempts_ = 0;
  PipeObservation last_observation_;
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  bool manual_override_enabled_ = false;
  int16_t manual_x_ = 0;
  int16_t manual_y_ = 0;
  int16_t manual_z_ = 500;
  int16_t manual_r_ = 0;
  rclcpp::Time last_manual_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr manual_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr output_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_log_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_mode_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manual_control_service_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::PipeTrackerNode)
