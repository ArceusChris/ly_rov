#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ai_msgs/msg/perception_targets.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rov_pipe_tracker
{

enum class WebStreamKind
{
  Raw,
  Overlay,
  Processed,
};

class PipeWebUiNode : public rclcpp::Node
{
public:
  explicit PipeWebUiNode(const rclcpp::NodeOptions & options)
  : Node("pipe_web_ui", options)
  {
    raw_topic_ = declare_parameter<std::string>("raw_topic", "image_raw");
    mask_topic_ = declare_parameter<std::string>("mask_topic", "pipe_cv_segmentation");
    processed_topic_ = declare_parameter<std::string>("processed_topic", "remote_processed_image");
    overlay_stream_url_ = declare_parameter<std::string>("overlay_stream_url", "");
    processed_stream_url_ = declare_parameter<std::string>("processed_stream_url", "");
    command_log_topic_ =
      declare_parameter<std::string>("command_log_topic", "pipe_tracker_command_log");
    http_host_ = declare_parameter<std::string>("http_host", "0.0.0.0");
    http_port_ = declare_parameter<int>("http_port", 8080);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    stream_fps_ = declare_parameter<double>("stream_fps", 15.0);
    mask_label_ = declare_parameter<int>("mask_label", -1);
    arm_service_name_ = declare_parameter<std::string>("arm_service", "/pipe_tracker/arm");
    manual_mode_service_name_ =
      declare_parameter<std::string>("manual_mode_service", "/pipe_tracker/manual_mode");
    manual_control_service_name_ = declare_parameter<std::string>(
      "manual_control_service", "/pipe_tracker/manual_control_enabled");

    raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PipeWebUiNode::on_image, this, std::placeholders::_1));
    mask_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      mask_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PipeWebUiNode::on_masks, this, std::placeholders::_1));
    if (processed_stream_url_.empty() && !processed_topic_.empty()) {
      processed_sub_ = create_subscription<sensor_msgs::msg::Image>(
        processed_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PipeWebUiNode::on_processed_image, this, std::placeholders::_1));
    }
    command_log_sub_ = create_subscription<std_msgs::msg::String>(
      command_log_topic_, rclcpp::QoS(50),
      std::bind(&PipeWebUiNode::on_command_log, this, std::placeholders::_1));

    arm_client_ = create_client<std_srvs::srv::SetBool>(arm_service_name_);
    manual_mode_client_ = create_client<std_srvs::srv::Trigger>(manual_mode_service_name_);
    manual_control_client_ = create_client<std_srvs::srv::SetBool>(manual_control_service_name_);

    start_server();
  }

  ~PipeWebUiNode() override
  {
    running_ = false;
    frame_cv_.notify_all();
    if (listen_fd_ >= 0) {
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    for (auto & thread : client_threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

private:
  void start_server()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      throw std::runtime_error("failed to create web UI socket");
    }

    int yes = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(http_port_));
    if (http_host_ == "0.0.0.0") {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, http_host_.c_str(), &addr.sin_addr) != 1) {
      throw std::runtime_error("invalid http_host: " + http_host_);
    }

    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      const std::string error = std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      throw std::runtime_error("failed to bind web UI socket: " + error);
    }
    if (::listen(listen_fd_, 16) != 0) {
      const std::string error = std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      throw std::runtime_error("failed to listen on web UI socket: " + error);
    }

    running_ = true;
    server_thread_ = std::thread([this]() { accept_loop(); });
    RCLCPP_INFO(
      get_logger(), "pipe web UI listening on http://%s:%d",
      http_host_.c_str(), http_port_);
  }

  void accept_loop()
  {
    while (running_) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      const int client_fd = ::accept(
        listen_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
      if (client_fd < 0) {
        if (running_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "web UI accept failed: %s", std::strerror(errno));
        }
        continue;
      }
      client_threads_.emplace_back([this, client_fd]() { handle_client(client_fd); });
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat frame;
    if (!to_bgr(*msg, frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "unsupported image encoding for web UI: %s",
        msg->encoding.c_str());
      return;
    }
    if (!frame.isContinuous()) {
      frame = frame.clone();
    }

    std::vector<uint8_t> raw_jpeg;
    if (!encode_jpeg(frame, raw_jpeg)) {
      return;
    }

    cv::Mat mask;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_bgr_ = frame.clone();
      raw_jpeg_ = std::move(raw_jpeg);
      ++raw_seq_;
      if (!latest_mask_.empty()) {
        mask = latest_mask_.clone();
      }
    }
    if (!mask.empty()) {
      publish_overlay(frame, mask);
    }
    frame_cv_.notify_all();
  }

  void on_masks(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    cv::Mat best_mask;
    size_t best_pixels = 0;
    for (const auto & target : msg->targets) {
      for (const auto & capture : target.captures) {
        if (capture.img.width == 0 || capture.img.height == 0) {
          continue;
        }
        const size_t pixel_count =
          static_cast<size_t>(capture.img.width) * static_cast<size_t>(capture.img.height);
        if (capture.features.size() < pixel_count) {
          continue;
        }

        cv::Mat mask(
          static_cast<int>(capture.img.height), static_cast<int>(capture.img.width), CV_8UC1);
        size_t active = 0;
        for (size_t i = 0; i < pixel_count; ++i) {
          const float value = capture.features[i];
          const bool selected = (mask_label_ < 0 && value != 0.0f) ||
            value == static_cast<float>(mask_label_);
          mask.data[i] = selected ? 255 : 0;
          if (selected) {
            ++active;
          }
        }
        if (active > best_pixels) {
          best_pixels = active;
          best_mask = mask.clone();
        }
      }
    }
    if (best_mask.empty()) {
      return;
    }

    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_mask_ = best_mask.clone();
      if (!latest_bgr_.empty()) {
        frame = latest_bgr_.clone();
      }
    }
    if (!frame.empty()) {
      publish_overlay(frame, best_mask);
    }
  }

  void on_command_log(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_log_mutex_);
    command_log_.push_back(msg->data);
    if (command_log_.size() > max_command_log_lines_) {
      command_log_.erase(command_log_.begin(), command_log_.begin() +
        static_cast<std::ptrdiff_t>(command_log_.size() - max_command_log_lines_));
    }
    ++command_log_seq_;
  }

  void on_processed_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat frame;
    if (!to_bgr(*msg, frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "unsupported processed image encoding for web UI: %s",
        msg->encoding.c_str());
      return;
    }
    if (!frame.isContinuous()) {
      frame = frame.clone();
    }

    std::vector<uint8_t> jpeg;
    if (!encode_jpeg(frame, jpeg)) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      processed_jpeg_ = std::move(jpeg);
      ++processed_seq_;
    }
    frame_cv_.notify_all();
  }

  void publish_overlay(const cv::Mat & frame, const cv::Mat & mask)
  {
    cv::Mat resized_mask;
    if (mask.size() == frame.size()) {
      resized_mask = mask;
    } else {
      cv::resize(mask, resized_mask, frame.size(), 0.0, 0.0, cv::INTER_NEAREST);
    }

    cv::Mat binary;
    cv::compare(resized_mask, 0, binary, cv::CMP_GT);

    cv::Mat tint = frame.clone();
    tint.setTo(cv::Scalar(0, 255, 80), binary);

    cv::Mat overlay;
    cv::addWeighted(frame, 0.68, tint, 0.32, 0.0, overlay);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(overlay, contours, -1, cv::Scalar(0, 0, 255), 2);

    std::vector<uint8_t> overlay_jpeg;
    if (!encode_jpeg(overlay, overlay_jpeg)) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      overlay_jpeg_ = std::move(overlay_jpeg);
      ++overlay_seq_;
    }
    frame_cv_.notify_all();
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
    if (msg.encoding == "mono8") {
      cv::Mat mono(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC1,
        const_cast<uint8_t *>(msg.data.data()), msg.step);
      cv::cvtColor(mono, frame, cv::COLOR_GRAY2BGR);
      return true;
    }
    return false;
  }

  bool encode_jpeg(const cv::Mat & frame, std::vector<uint8_t> & jpeg)
  {
    std::vector<int> params = {
      cv::IMWRITE_JPEG_QUALITY, std::clamp(jpeg_quality_, 1, 100)
    };
    std::vector<uchar> encoded;
    if (!cv::imencode(".jpg", frame, encoded, params)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "failed to encode web UI JPEG");
      return false;
    }
    jpeg.assign(encoded.begin(), encoded.end());
    return true;
  }

  void handle_client(int client_fd)
  {
    std::string request;
    request.resize(4096);
    const ssize_t received = ::recv(client_fd, request.data(), request.size() - 1, 0);
    if (received <= 0) {
      ::close(client_fd);
      return;
    }
    request.resize(static_cast<size_t>(received));

    std::istringstream stream(request);
    std::string method;
    std::string target;
    std::string version;
    stream >> method >> target >> version;
    const auto query_pos = target.find('?');
    const std::string path = target.substr(0, query_pos);

    if (method == "GET" && path == "/") {
      send_response(client_fd, "200 OK", "text/html; charset=utf-8", page_html());
    } else if (method == "GET" && path == "/raw.mjpg") {
      stream_mjpeg(client_fd, WebStreamKind::Raw);
    } else if (method == "GET" && (path == "/overlay.mjpg" || path == "/mask.mjpg")) {
      stream_mjpeg(client_fd, WebStreamKind::Overlay);
    } else if (method == "GET" && path == "/processed.mjpg") {
      stream_mjpeg(client_fd, WebStreamKind::Processed);
    } else if (method == "GET" && path == "/api/status") {
      send_response(client_fd, "200 OK", "application/json", status_json());
    } else if (method == "GET" && path == "/api/command_log") {
      send_response(client_fd, "200 OK", "application/json", command_log_json());
    } else if (method == "POST" && path == "/api/arm") {
      send_response(client_fd, "200 OK", "application/json", call_set_bool(arm_client_, true));
    } else if (method == "POST" && path == "/api/disarm") {
      send_response(client_fd, "200 OK", "application/json", call_set_bool(arm_client_, false));
    } else if (method == "POST" && path == "/api/manual_mode") {
      send_response(client_fd, "200 OK", "application/json", call_trigger(manual_mode_client_));
    } else if (method == "POST" && path == "/api/manual_control/on") {
      send_response(
        client_fd, "200 OK", "application/json", call_set_bool(manual_control_client_, true));
    } else if (method == "POST" && path == "/api/manual_control/off") {
      send_response(
        client_fd, "200 OK", "application/json", call_set_bool(manual_control_client_, false));
    } else {
      send_response(client_fd, "404 Not Found", "text/plain", "not found\n");
    }
    ::close(client_fd);
  }

  void stream_mjpeg(int client_fd, WebStreamKind kind)
  {
    const std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Cache-Control: no-store\r\n"
      "Pragma: no-cache\r\n"
      "Connection: close\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    if (!send_all(client_fd, header.data(), header.size())) {
      return;
    }

    uint64_t last_seq = 0;
    const auto frame_interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / std::max(1.0, stream_fps_)));
    auto last_sent = std::chrono::steady_clock::now() - frame_interval;

    while (running_) {
      std::vector<uint8_t> jpeg;
      uint64_t seq = 0;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait_for(lock, std::chrono::seconds(1), [&]() {
          return !running_ || stream_seq(kind) != last_seq;
        });
        if (!running_) {
          break;
        }
        jpeg = stream_jpeg(kind);
        seq = stream_seq(kind);
      }
      if (jpeg.empty()) {
        continue;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - last_sent < frame_interval) {
        std::this_thread::sleep_for(frame_interval - (now - last_sent));
      }
      last_sent = std::chrono::steady_clock::now();

      std::ostringstream part;
      part << "--frame\r\n"
           << "Content-Type: image/jpeg\r\n"
           << "Content-Length: " << jpeg.size() << "\r\n\r\n";
      const std::string part_header = part.str();
      if (!send_all(client_fd, part_header.data(), part_header.size()) ||
        !send_all(client_fd, jpeg.data(), jpeg.size()) ||
        !send_all(client_fd, "\r\n", 2))
      {
        break;
      }
      last_seq = seq;
    }
  }

  uint64_t stream_seq(WebStreamKind kind) const
  {
    if (kind == WebStreamKind::Overlay) {
      return overlay_seq_;
    }
    if (kind == WebStreamKind::Processed) {
      return processed_seq_;
    }
    return raw_seq_;
  }

  std::vector<uint8_t> stream_jpeg(WebStreamKind kind) const
  {
    if (kind == WebStreamKind::Overlay) {
      return overlay_jpeg_;
    }
    if (kind == WebStreamKind::Processed) {
      return processed_jpeg_;
    }
    return raw_jpeg_;
  }

  std::string call_set_bool(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client, bool value)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
      return make_json(false, "service unavailable");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = value;
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return make_json(false, "service timeout");
    }
    const auto response = future.get();
    return make_json(response->success, response->message);
  }

  std::string call_trigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
      return make_json(false, "service unavailable");
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return make_json(false, "service timeout");
    }
    const auto response = future.get();
    return make_json(response->success, response->message);
  }

  std::string status_json() const
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    std::ostringstream out;
    out << "{\"raw_frames\":" << raw_seq_
        << ",\"overlay_frames\":" << overlay_seq_
        << ",\"processed_frames\":" << processed_seq_
        << ",\"has_raw\":" << (raw_jpeg_.empty() ? "false" : "true")
        << ",\"has_overlay\":" << (overlay_jpeg_.empty() ? "false" : "true")
        << ",\"has_processed\":" << (processed_jpeg_.empty() ? "false" : "true")
        << "}";
    return out.str();
  }

  std::string command_log_json() const
  {
    std::lock_guard<std::mutex> lock(command_log_mutex_);
    std::ostringstream out;
    out << "{\"seq\":" << command_log_seq_ << ",\"lines\":[";
    for (size_t i = 0; i < command_log_.size(); ++i) {
      if (i != 0) {
        out << ",";
      }
      out << "\"" << json_escape(command_log_[i]) << "\"";
    }
    out << "]}";
    return out.str();
  }

  static std::string make_json(bool ok, const std::string & message)
  {
    return std::string("{\"ok\":") + (ok ? "true" : "false") +
      ",\"message\":\"" + json_escape(message) + "\"}";
  }

  static std::string json_escape(const std::string & input)
  {
    std::string escaped;
    escaped.reserve(input.size());
    for (const char c : input) {
      if (c == '"' || c == '\\') {
        escaped.push_back('\\');
      }
      escaped.push_back(c);
    }
    return escaped;
  }

  static bool send_all(int fd, const void * data, size_t size)
  {
    const auto * bytes = static_cast<const uint8_t *>(data);
    size_t sent_total = 0;
    while (sent_total < size) {
      const ssize_t sent = ::send(fd, bytes + sent_total, size - sent_total, MSG_NOSIGNAL);
      if (sent <= 0) {
        return false;
      }
      sent_total += static_cast<size_t>(sent);
    }
    return true;
  }

  static void send_response(
    int fd, const std::string & status, const std::string & content_type,
    const std::string & body)
  {
    std::ostringstream header;
    header << "HTTP/1.1 " << status << "\r\n"
           << "Content-Type: " << content_type << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Cache-Control: no-store\r\n"
           << "Connection: close\r\n\r\n";
    const std::string header_text = header.str();
    send_all(fd, header_text.data(), header_text.size());
    send_all(fd, body.data(), body.size());
  }

  std::string page_html() const
  {
    const std::string overlay_src =
      overlay_stream_url_.empty() ? "/overlay.mjpg" : overlay_stream_url_;
    const std::string processed_src =
      processed_stream_url_.empty() ? "/processed.mjpg" : processed_stream_url_;
    return std::string(R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pipe Follow Web UI</title>
<style>
:root { color-scheme: dark; font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
* { box-sizing: border-box; }
body { margin: 0; background: #111418; color: #eef2f4; }
header { display: flex; align-items: center; justify-content: space-between; gap: 16px; padding: 16px 20px; border-bottom: 1px solid #2a3037; }
h1 { margin: 0; font-size: 20px; font-weight: 650; letter-spacing: 0; }
main { display: grid; grid-template-columns: minmax(0, 1fr) 340px; gap: 16px; padding: 16px; min-height: calc(100vh - 65px); }
.feeds { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 16px; align-content: start; }
.panel { border: 1px solid #2a3037; border-radius: 8px; background: #171b20; overflow: hidden; }
.panel h2 { margin: 0; padding: 10px 12px; font-size: 14px; font-weight: 600; color: #bac4cc; border-bottom: 1px solid #2a3037; }
.viewport { aspect-ratio: 4 / 3; background: #050608; display: grid; place-items: center; }
img { display: block; width: 100%; height: 100%; object-fit: contain; }
.controls { display: flex; flex-direction: column; gap: 10px; align-self: start; }
button { min-height: 42px; border: 1px solid #3a424b; border-radius: 7px; background: #222933; color: #eef2f4; font-size: 14px; font-weight: 650; cursor: pointer; }
button:hover { background: #2b3440; }
button.primary { background: #176b54; border-color: #21906f; }
button.danger { background: #702524; border-color: #a33c38; }
button:disabled { opacity: .62; cursor: wait; }
#status { min-height: 38px; padding: 10px 12px; border-radius: 7px; background: #0d1014; color: #bac4cc; font-size: 13px; line-height: 1.35; }
#motionState { min-height: 76px; padding: 10px 12px; border-radius: 7px; background: #0d1014; color: #eef2f4; font-size: 14px; line-height: 1.45; }
.log-panel { min-height: 260px; }
#commandLog { margin: 0; height: 260px; overflow: auto; padding: 10px 12px; background: #0d1014; color: #c8d4dc; font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; white-space: pre-wrap; }
@media (max-width: 900px) {
  main { grid-template-columns: 1fr; }
  .feeds { grid-template-columns: 1fr; }
}
@media (min-width: 901px) and (max-width: 1320px) {
  .feeds { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}
</style>
</head>
<body>
<header><h1>Pipe Follow Web UI</h1><span id="health">Connecting</span></header>
<main>
  <section class="feeds">
    <article class="panel"><h2>Raw Camera</h2><div class="viewport"><img src="/raw.mjpg" alt="Raw camera stream"></div></article>
    <article class="panel"><h2>Pipe Overlay</h2><div class="viewport"><img src=")HTML") +
      html_escape(overlay_src) + R"HTML(" alt="Pipe overlay stream"></div></article>
    <article class="panel"><h2>Remote Processing</h2><div class="viewport"><img src=")HTML" +
      html_escape(processed_src) + R"HTML(" alt="Remote processed stream"></div></article>
  </section>
  <aside class="controls">
    <section class="panel"><h2>ROV State</h2><div id="motionState">Waiting...</div></section>
    <button class="primary" data-action="/api/arm">ARM</button>
    <button class="danger" data-action="/api/disarm">DISARM</button>
    <button data-action="/api/manual_mode">MANUAL MODE</button>
    <button data-action="/api/manual_control/on">MANUAL CONTROL ON</button>
    <button data-action="/api/manual_control/off">MANUAL CONTROL OFF</button>
    <div id="status">Ready</div>
    <section class="panel log-panel"><h2>Command Log</h2><pre id="commandLog">Waiting...</pre></section>
  </aside>
</main>
<script>
const statusEl = document.getElementById('status');
const healthEl = document.getElementById('health');
const commandLogEl = document.getElementById('commandLog');
const motionStateEl = document.getElementById('motionState');
let commandLogSeq = 0;
async function postAction(path, button) {
  button.disabled = true;
  statusEl.textContent = 'Sending...';
  try {
    const response = await fetch(path, { method: 'POST' });
    const data = await response.json();
    statusEl.textContent = data.message || (data.ok ? 'OK' : 'Failed');
  } catch (error) {
    statusEl.textContent = String(error);
  } finally {
    button.disabled = false;
  }
}
document.querySelectorAll('button[data-action]').forEach((button) => {
  button.addEventListener('click', () => postAction(button.dataset.action, button));
});
function logField(line, name) {
  const match = line.match(new RegExp(`${name}=([^ ]+)`));
  return match ? match[1] : '';
}
function axisParts(x, y, z, r) {
  const parts = [];
  if (x > 0) parts.push('forward');
  if (x < 0) parts.push('backward');
  if (y > 0) parts.push('right');
  if (y < 0) parts.push('left');
  if (z > 520) parts.push('up');
  if (z < 480) parts.push('down');
  if (r > 0) parts.push('yaw right');
  if (r < 0) parts.push('yaw left');
  return parts;
}
function describeCommand(line) {
  if (!line) return 'Waiting for command data.';
  const mode = logField(line, 'mode');
  const reason = logField(line, 'reason');
  const x = Number(logField(line, 'x') || 0);
  const y = Number(logField(line, 'y') || 0);
  const z = Number(logField(line, 'z') || 500);
  const r = Number(logField(line, 'r') || 0);
  const parts = axisParts(x, y, z, r);
  const axes = `x=${x} y=${y} z=${z} r=${r}`;
  if (mode === 'disabled') return `Control stream is off. Current command: ${axes}.`;
  if (mode === 'neutral') return `ROV is holding neutral. Reason: ${reason || 'neutral protection'}. Current command: ${axes}.`;
  if (mode === 'manual_override') {
    return parts.length ? `Manual override is active: ${parts.join(', ')}. Current command: ${axes}.` : `Manual override is active. ROV is holding neutral. Current command: ${axes}.`;
  }
  if (mode === 'auto') {
    return parts.length ? `Automatic pipe following is active: ${parts.join(', ')}. Current command: ${axes}.` : `Automatic pipe following is active. ROV is holding neutral. Current command: ${axes}.`;
  }
  return `Latest command: ${line}`;
}
async function poll() {
  try {
    const response = await fetch('/api/status');
    const data = await response.json();
    healthEl.textContent = `Raw ${data.raw_frames} / Overlay ${data.overlay_frames} / Remote ${data.processed_frames}`;
  } catch (error) {
    healthEl.textContent = 'Disconnected';
  }
}
async function pollCommandLog() {
  try {
    const response = await fetch('/api/command_log');
    const data = await response.json();
    if (data.seq !== commandLogSeq) {
      commandLogSeq = data.seq;
      commandLogEl.textContent = data.lines.length ? data.lines.join('\n') : 'Waiting...';
      commandLogEl.scrollTop = commandLogEl.scrollHeight;
      motionStateEl.textContent = describeCommand(data.lines[data.lines.length - 1] || '');
    }
  } catch (error) {
    commandLogEl.textContent = String(error);
    motionStateEl.textContent = String(error);
  }
}
setInterval(poll, 1000);
setInterval(pollCommandLog, 500);
poll();
pollCommandLog();
</script>
</body>
</html>
)HTML";
  }

  static std::string html_escape(const std::string & input)
  {
    std::string escaped;
    escaped.reserve(input.size());
    for (const char c : input) {
      if (c == '&') {
        escaped += "&amp;";
      } else if (c == '"') {
        escaped += "&quot;";
      } else if (c == '<') {
        escaped += "&lt;";
      } else if (c == '>') {
        escaped += "&gt;";
      } else {
        escaped.push_back(c);
      }
    }
    return escaped;
  }

  std::string raw_topic_;
  std::string mask_topic_;
  std::string processed_topic_;
  std::string overlay_stream_url_;
  std::string processed_stream_url_;
  std::string command_log_topic_;
  std::string http_host_;
  int http_port_;
  int jpeg_quality_;
  double stream_fps_;
  int mask_label_;
  std::string arm_service_name_;
  std::string manual_mode_service_name_;
  std::string manual_control_service_name_;

  std::atomic<bool> running_{false};
  int listen_fd_ = -1;
  std::thread server_thread_;
  std::vector<std::thread> client_threads_;

  mutable std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  cv::Mat latest_bgr_;
  cv::Mat latest_mask_;
  std::vector<uint8_t> raw_jpeg_;
  std::vector<uint8_t> overlay_jpeg_;
  std::vector<uint8_t> processed_jpeg_;
  uint64_t raw_seq_ = 0;
  uint64_t overlay_seq_ = 0;
  uint64_t processed_seq_ = 0;

  mutable std::mutex command_log_mutex_;
  std::vector<std::string> command_log_;
  uint64_t command_log_seq_ = 0;
  static constexpr size_t max_command_log_lines_ = 120;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr processed_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr mask_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_log_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr manual_mode_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_control_client_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::PipeWebUiNode)
