#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rov_pipe_tracker
{

class ManualControlWebUiNode : public rclcpp::Node
{
public:
  explicit ManualControlWebUiNode(const rclcpp::NodeOptions & options)
  : Node("manual_control_web_ui", options)
  {
    command_topic_ = declare_parameter<std::string>("command_topic", "manual_control_command");
    http_host_ = declare_parameter<std::string>("http_host", "0.0.0.0");
    http_port_ = declare_parameter<int>("http_port", 8081);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    arm_service_name_ = declare_parameter<std::string>("arm_service", "/pipe_tracker/arm");
    manual_mode_service_name_ =
      declare_parameter<std::string>("manual_mode_service", "/pipe_tracker/manual_mode");
    manual_control_service_name_ = declare_parameter<std::string>(
      "manual_control_service", "/pipe_tracker/manual_control_enabled");

    command_pub_ =
      create_publisher<std_msgs::msg::Int16MultiArray>(command_topic_, rclcpp::QoS(10));
    arm_client_ = create_client<std_srvs::srv::SetBool>(arm_service_name_);
    manual_mode_client_ = create_client<std_srvs::srv::Trigger>(manual_mode_service_name_);
    manual_control_client_ = create_client<std_srvs::srv::SetBool>(manual_control_service_name_);

    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_))),
      std::bind(&ManualControlWebUiNode::publish_command, this));

    start_server();
  }

  ~ManualControlWebUiNode() override
  {
    running_ = false;
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
      throw std::runtime_error("failed to create manual control web UI socket");
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
      throw std::runtime_error("failed to bind manual control web UI socket: " + error);
    }
    if (::listen(listen_fd_, 16) != 0) {
      const std::string error = std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      throw std::runtime_error("failed to listen on manual control web UI socket: " + error);
    }

    running_ = true;
    server_thread_ = std::thread([this]() { accept_loop(); });
    RCLCPP_INFO(
      get_logger(), "manual control web UI listening on http://%s:%d",
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
            get_logger(), *get_clock(), 2000,
            "manual control web UI accept failed: %s", std::strerror(errno));
        }
        continue;
      }
      client_threads_.emplace_back([this, client_fd]() { handle_client(client_fd); });
    }
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
    const std::string query =
      query_pos == std::string::npos ? std::string() : target.substr(query_pos + 1);

    if (method == "GET" && path == "/") {
      send_response(client_fd, "200 OK", "text/html; charset=utf-8", page_html());
    } else if (method == "GET" && path == "/api/status") {
      send_response(client_fd, "200 OK", "application/json", status_json());
    } else if (method == "POST" && path == "/api/command") {
      set_command(
        query_int(query, "x", 0),
        query_int(query, "y", 0),
        query_int(query, "z", 500),
        query_int(query, "r", 0),
        true);
      send_response(client_fd, "200 OK", "application/json", status_json());
    } else if (method == "POST" && path == "/api/stop") {
      set_command(0, 0, 500, 0, true);
      send_response(client_fd, "200 OK", "application/json", status_json());
    } else if (method == "POST" && path == "/api/release") {
      set_command(0, 0, 500, 0, false);
      send_response(client_fd, "200 OK", "application/json", status_json());
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

  void set_command(int x, int y, int z, int r, bool override_enabled)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      x_ = clamp_axis(x);
      y_ = clamp_axis(y);
      z_ = static_cast<int16_t>(std::clamp(z, 0, 1000));
      r_ = clamp_axis(r);
      override_enabled_ = override_enabled;
      last_command_time_ = std::chrono::steady_clock::now();
      ++command_seq_;
    }
    publish_command();
  }

  void publish_command()
  {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 500;
    int16_t r = 0;
    bool override_enabled = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (override_enabled_ && command_timeout_s_ > 0.0) {
        const auto age = std::chrono::steady_clock::now() - last_command_time_;
        if (age > std::chrono::duration<double>(command_timeout_s_)) {
          x_ = 0;
          y_ = 0;
          z_ = 500;
          r_ = 0;
        }
      }
      x = x_;
      y = y_;
      z = z_;
      r = r_;
      override_enabled = override_enabled_;
    }

    std_msgs::msg::Int16MultiArray msg;
    msg.data = {x, y, z, r, static_cast<int16_t>(override_enabled ? 1 : 0)};
    command_pub_->publish(msg);
  }

  static int16_t clamp_axis(int value)
  {
    return static_cast<int16_t>(std::clamp(value, -1000, 1000));
  }

  static int query_int(const std::string & query, const std::string & key, int default_value)
  {
    size_t pos = 0;
    while (pos < query.size()) {
      const size_t next = query.find('&', pos);
      const std::string item = query.substr(pos, next == std::string::npos ? next : next - pos);
      const size_t eq = item.find('=');
      if (eq != std::string::npos && item.substr(0, eq) == key) {
        try {
          return std::stoi(item.substr(eq + 1));
        } catch (const std::exception &) {
          return default_value;
        }
      }
      if (next == std::string::npos) {
        break;
      }
      pos = next + 1;
    }
    return default_value;
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
    std::lock_guard<std::mutex> lock(command_mutex_);
    std::ostringstream out;
    out << "{\"override_enabled\":" << (override_enabled_ ? "true" : "false")
        << ",\"x\":" << x_
        << ",\"y\":" << y_
        << ",\"z\":" << z_
        << ",\"r\":" << r_
        << ",\"seq\":" << command_seq_
        << ",\"motion_text\":\"" << json_escape(motion_text_locked()) << "\""
        << "}";
    return out.str();
  }

  std::string motion_text_locked() const
  {
    if (!override_enabled_) {
      return "Manual override released. Automatic pipe following may control the ROV.";
    }

    std::vector<std::string> parts;
    if (x_ > 0) {
      parts.emplace_back("forward");
    } else if (x_ < 0) {
      parts.emplace_back("backward");
    }
    if (y_ > 0) {
      parts.emplace_back("right");
    } else if (y_ < 0) {
      parts.emplace_back("left");
    }
    if (z_ > 520) {
      parts.emplace_back("up");
    } else if (z_ < 480) {
      parts.emplace_back("down");
    }
    if (r_ > 0) {
      parts.emplace_back("yaw right");
    } else if (r_ < 0) {
      parts.emplace_back("yaw left");
    }

    if (parts.empty()) {
      return "Manual override active. Neutral command: x=0 y=0 z=500 r=0.";
    }

    std::ostringstream text;
    text << "Manual override active. Commanding: ";
    for (size_t i = 0; i < parts.size(); ++i) {
      if (i != 0) {
        text << ", ";
      }
      text << parts[i];
    }
    text << ". Axes: x=" << x_ << " y=" << y_ << " z=" << z_ << " r=" << r_ << ".";
    return text.str();
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

  static std::string page_html()
  {
    return R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Manual ROV Control</title>
<style>
:root { color-scheme: dark; font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
* { box-sizing: border-box; }
body { margin: 0; background: #101214; color: #eef2f4; }
header { display: flex; align-items: center; justify-content: space-between; gap: 16px; padding: 14px 18px; border-bottom: 1px solid #2e353d; }
h1 { margin: 0; font-size: 20px; font-weight: 650; letter-spacing: 0; }
main { display: grid; grid-template-columns: minmax(260px, 1fr) 320px; gap: 18px; padding: 18px; }
.pad { display: grid; grid-template-columns: repeat(3, minmax(74px, 1fr)); grid-auto-rows: 72px; gap: 10px; align-content: start; }
.panel { border: 1px solid #2e353d; border-radius: 8px; background: #181d22; padding: 14px; }
.panel h2 { margin: 0 0 12px; font-size: 14px; color: #bac4cc; font-weight: 650; }
button { border: 1px solid #3b4650; border-radius: 7px; background: #242c35; color: #eef2f4; font-size: 15px; font-weight: 700; cursor: pointer; touch-action: none; }
button:hover { background: #303a45; }
button:active, button.active { background: #1f755d; border-color: #2da37f; }
button.stop { background: #7a2927; border-color: #b1413c; }
button.release { background: #244261; border-color: #386592; }
.side { display: flex; flex-direction: column; gap: 12px; }
.stack { display: grid; gap: 8px; }
.row { display: grid; grid-template-columns: 92px 1fr 52px; gap: 10px; align-items: center; color: #bac4cc; font-size: 13px; }
input[type=range] { width: 100%; }
#status { min-height: 46px; padding: 10px; border-radius: 7px; background: #0b0d10; color: #c6d0d8; font-size: 13px; line-height: 1.4; }
#motionState { min-height: 92px; padding: 12px; border-radius: 7px; background: #0b0d10; color: #eef2f4; font-size: 15px; line-height: 1.45; }
@media (max-width: 760px) {
  main { grid-template-columns: 1fr; }
  .pad { grid-auto-rows: 66px; }
}
</style>
</head>
<body>
<header><h1>Manual ROV Control</h1><span id="health">Disconnected</span></header>
<main>
  <section class="panel">
    <h2>Motion</h2>
    <div class="pad">
      <span></span><button data-move="forward">FWD</button><span></span>
      <button data-move="left">LEFT</button><button class="stop" data-action="/api/stop">STOP</button><button data-move="right">RIGHT</button>
      <button data-move="yaw_left">YAW L</button><button data-move="back">BACK</button><button data-move="yaw_right">YAW R</button>
      <span></span><button data-move="up">UP</button><span></span>
      <span></span><button data-move="down">DOWN</button><span></span>
    </div>
  </section>
  <aside class="side">
    <section class="panel stack">
      <h2>Axis Limits</h2>
      <label class="row"><span>Move</span><input id="move" type="range" min="0" max="500" value="120"><output id="moveOut">120</output></label>
      <label class="row"><span>Yaw</span><input id="yaw" type="range" min="0" max="500" value="180"><output id="yawOut">180</output></label>
      <label class="row"><span>Vertical</span><input id="vertical" type="range" min="0" max="400" value="100"><output id="verticalOut">100</output></label>
    </section>
    <section class="panel stack">
      <h2>Vehicle</h2>
      <button data-action="/api/manual_mode">MANUAL MODE</button>
      <button data-action="/api/arm">ARM</button>
      <button class="stop" data-action="/api/disarm">DISARM</button>
      <button data-action="/api/manual_control/on">CONTROL ON</button>
      <button data-action="/api/manual_control/off">CONTROL OFF</button>
      <button class="release" data-action="/api/release">RELEASE AUTO</button>
    </section>
    <section class="panel">
      <h2>Motion State</h2>
      <div id="motionState">Waiting for command state.</div>
    </section>
    <div id="status">Ready</div>
  </aside>
</main>
<script>
const statusEl = document.getElementById('status');
const healthEl = document.getElementById('health');
const motionStateEl = document.getElementById('motionState');
const sliders = {
  move: document.getElementById('move'),
  yaw: document.getElementById('yaw'),
  vertical: document.getElementById('vertical')
};
const outputs = {
  move: document.getElementById('moveOut'),
  yaw: document.getElementById('yawOut'),
  vertical: document.getElementById('verticalOut')
};
for (const name of Object.keys(sliders)) {
  const update = () => { outputs[name].textContent = sliders[name].value; };
  sliders[name].addEventListener('input', update);
  update();
}
function axesFor(moveName) {
  const move = Number(sliders.move.value);
  const yaw = Number(sliders.yaw.value);
  const vertical = Number(sliders.vertical.value);
  const axes = { x: 0, y: 0, z: 500, r: 0 };
  if (moveName === 'forward') axes.x = move;
  if (moveName === 'back') axes.x = -move;
  if (moveName === 'left') axes.y = -move;
  if (moveName === 'right') axes.y = move;
  if (moveName === 'yaw_left') axes.r = -yaw;
  if (moveName === 'yaw_right') axes.r = yaw;
  if (moveName === 'up') axes.z = 500 + vertical;
  if (moveName === 'down') axes.z = 500 - vertical;
  return axes;
}
async function post(path) {
  try {
    const response = await fetch(path, { method: 'POST' });
    const data = await response.json();
    statusEl.textContent = JSON.stringify(data);
    if (data.motion_text) motionStateEl.textContent = data.motion_text;
    return data;
  } catch (error) {
    statusEl.textContent = String(error);
    return null;
  }
}
function commandPath(axes) {
  return `/api/command?x=${axes.x}&y=${axes.y}&z=${axes.z}&r=${axes.r}`;
}
let holdTimer = null;
let activeButton = null;
function startHold(button, moveName) {
  stopHold(false);
  activeButton = button;
  button.classList.add('active');
  const send = () => post(commandPath(axesFor(moveName)));
  send();
  holdTimer = setInterval(send, 150);
}
function stopHold(sendStop = true) {
  if (holdTimer) clearInterval(holdTimer);
  holdTimer = null;
  if (activeButton) activeButton.classList.remove('active');
  activeButton = null;
  if (sendStop) post('/api/stop');
}
document.querySelectorAll('button[data-move]').forEach((button) => {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    startHold(button, button.dataset.move);
  });
  button.addEventListener('pointerup', () => stopHold(true));
  button.addEventListener('pointerleave', () => stopHold(true));
  button.addEventListener('pointercancel', () => stopHold(true));
});
document.querySelectorAll('button[data-action]').forEach((button) => {
  button.addEventListener('click', () => post(button.dataset.action));
});
async function poll() {
  try {
    const response = await fetch('/api/status');
    const data = await response.json();
    healthEl.textContent = data.override_enabled ? `MANUAL x=${data.x} y=${data.y} z=${data.z} r=${data.r}` : 'Released';
    motionStateEl.textContent = data.motion_text || 'No motion state.';
  } catch (error) {
    healthEl.textContent = 'Disconnected';
    motionStateEl.textContent = String(error);
  }
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
)HTML";
  }

  std::string command_topic_;
  std::string http_host_;
  int http_port_;
  double publish_rate_hz_;
  double command_timeout_s_;
  std::string arm_service_name_;
  std::string manual_mode_service_name_;
  std::string manual_control_service_name_;

  std::atomic<bool> running_{false};
  int listen_fd_ = -1;
  std::thread server_thread_;
  std::vector<std::thread> client_threads_;

  mutable std::mutex command_mutex_;
  int16_t x_ = 0;
  int16_t y_ = 0;
  int16_t z_ = 500;
  int16_t r_ = 0;
  bool override_enabled_ = false;
  uint64_t command_seq_ = 0;
  std::chrono::steady_clock::time_point last_command_time_{std::chrono::steady_clock::now()};

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr command_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr manual_mode_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_control_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::ManualControlWebUiNode)
