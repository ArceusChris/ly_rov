#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ardupilotmega/mavlink.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rov_pipe_tracker
{

class MavlinkManualControlBridgeNode : public rclcpp::Node
{
public:
  explicit MavlinkManualControlBridgeNode(const rclcpp::NodeOptions & options)
  : Node("mavlink_manual_control_bridge", options)
  {
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "mavlink_manual_control_command");
    target_ip_ = declare_parameter<std::string>("target_ip", "127.0.0.1");
    target_port_ = declare_parameter<int>("target_port", 14550);
    udp_bind_port_ = declare_parameter<int>("udp_bind_port", 14550);
    learn_target_from_udp_ = declare_parameter<bool>("learn_target_from_udp", true);
    source_system_ = declare_parameter<int>("source_system", 255);
    source_component_ = declare_parameter<int>("source_component", MAV_COMP_ID_ONBOARD_COMPUTER);
    target_system_ = declare_parameter<int>("target_system", 1);
    target_component_ = declare_parameter<int>("target_component", MAV_COMP_ID_AUTOPILOT1);
    set_manual_mode_ = declare_parameter<bool>("set_manual_mode", true);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    manual_mode_ = declare_parameter<int>("manual_mode", 19);
    startup_command_retries_ = declare_parameter<int>("startup_command_retries", 5);
    startup_command_interval_s_ = declare_parameter<double>("startup_command_interval_s", 0.5);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 10.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    manual_control_enabled_ = declare_parameter<bool>("manual_control_enabled", true);

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

    command_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      command_topic_, rclcpp::QoS(10),
      std::bind(&MavlinkManualControlBridgeNode::on_command, this, std::placeholders::_1));
    arm_service_ = create_service<std_srvs::srv::SetBool>(
      "~/arm",
      std::bind(
        &MavlinkManualControlBridgeNode::on_arm_service, this, std::placeholders::_1,
        std::placeholders::_2));
    manual_mode_service_ = create_service<std_srvs::srv::Trigger>(
      "~/manual_mode",
      std::bind(
        &MavlinkManualControlBridgeNode::on_manual_mode_service, this, std::placeholders::_1,
        std::placeholders::_2));
    manual_control_service_ = create_service<std_srvs::srv::SetBool>(
      "~/manual_control_enabled",
      std::bind(
        &MavlinkManualControlBridgeNode::on_manual_control_service, this, std::placeholders::_1,
        std::placeholders::_2));

    if (learn_target_from_udp_) {
      receive_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MavlinkManualControlBridgeNode::receive_mavlink, this));
    }
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_))),
      std::bind(&MavlinkManualControlBridgeNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "MAVLink bridge command_topic=%s target fallback=%s:%d bind_port=%d "
      "learn_target_from_udp=%d",
      command_topic_.c_str(), target_ip_.c_str(), target_port_, udp_bind_port_,
      learn_target_from_udp_);
  }

  ~MavlinkManualControlBridgeNode() override
  {
    send_manual_control(0, 0, 500, 0);
    if (socket_ >= 0) {
      ::close(socket_);
    }
  }

private:
  void on_command(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MAVLink command requires [x, y, z, r]");
      return;
    }

    x_ = clamp_axis(msg->data[0]);
    y_ = clamp_axis(msg->data[1]);
    z_ = static_cast<int16_t>(std::clamp<int>(msg->data[2], 0, 1000));
    r_ = clamp_axis(msg->data[3]);
    last_command_time_ = now();
  }

  void tick()
  {
    send_heartbeat();
    send_startup_commands();

    if (!manual_control_enabled_) {
      return;
    }

    if (!vehicle_armed_ || vehicle_custom_mode_ != static_cast<uint32_t>(manual_mode_)) {
      send_manual_control(0, 0, 500, 0);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "neutral command: waiting for armed/manual mode armed=%d custom_mode=%u",
        vehicle_armed_, vehicle_custom_mode_);
      return;
    }

    if ((now() - last_command_time_).seconds() > command_timeout_s_) {
      send_manual_control(0, 0, 500, 0);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "neutral command: ROS2 command topic stale");
      return;
    }

    send_manual_control(x_, y_, z_, r_);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "MAVLink manual_control: x=%d y=%d z=%d r=%d", x_, y_, z_, r_);
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
      vehicle_armed_ = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      vehicle_custom_mode_ = heartbeat.custom_mode;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ArduSub heartbeat: armed=%d base_mode=0x%02x custom_mode=%u system_status=%u",
        vehicle_armed_, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
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
    manual_control_enabled_ = request->data;
    if (!manual_control_enabled_) {
      send_manual_control(0, 0, 500, 0);
    }
    response->success = true;
    response->message = manual_control_enabled_ ?
      "manual control stream enabled" : "manual control stream disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  std::string command_topic_;
  std::string target_ip_;
  int target_port_;
  int udp_bind_port_;
  bool learn_target_from_udp_;
  int source_system_;
  int source_component_;
  int target_system_;
  int target_component_;
  bool set_manual_mode_;
  bool auto_arm_;
  int manual_mode_;
  int startup_command_retries_;
  double startup_command_interval_s_;
  double command_rate_hz_;
  double command_timeout_s_;
  bool manual_control_enabled_;

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
  int16_t x_ = 0;
  int16_t y_ = 0;
  int16_t z_ = 500;
  int16_t r_ = 0;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_mode_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manual_control_service_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::MavlinkManualControlBridgeNode)
