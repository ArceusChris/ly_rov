#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace rov_pipe_tracker
{

enum class TunableType
{
  Integer,
  Double,
  Bool,
};

struct TunableParameter
{
  const char * group;
  const char * node_name;
  const char * name;
  TunableType type;
  double min_value;
  double max_value;
};

class ParameterTuningWebUiNode : public rclcpp::Node
{
public:
  explicit ParameterTuningWebUiNode(const rclcpp::NodeOptions & options)
  : Node("parameter_tuning_web_ui", options)
  {
    http_host_ = declare_parameter<std::string>("http_host", "0.0.0.0");
    http_port_ = declare_parameter<int>("http_port", 8082);
    segmenter_node_ = declare_parameter<std::string>("segmenter_node", "/cv_pipe_segmenter");
    tracker_node_ = declare_parameter<std::string>("tracker_node", "/pipe_tracker");

    cv_get_client_ =
      create_client<rcl_interfaces::srv::GetParameters>(segmenter_node_ + "/get_parameters");
    cv_set_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(segmenter_node_ + "/set_parameters");
    tracker_get_client_ =
      create_client<rcl_interfaces::srv::GetParameters>(tracker_node_ + "/get_parameters");
    tracker_set_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(tracker_node_ + "/set_parameters");

    start_server();
  }

  ~ParameterTuningWebUiNode() override
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
      throw std::runtime_error("failed to create parameter tuning web UI socket");
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
      throw std::runtime_error("failed to bind parameter tuning web UI socket: " + error);
    }
    if (::listen(listen_fd_, 16) != 0) {
      const std::string error = std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      throw std::runtime_error("failed to listen on parameter tuning web UI socket: " + error);
    }

    running_ = true;
    server_thread_ = std::thread([this]() { accept_loop(); });
    RCLCPP_INFO(
      get_logger(), "parameter tuning web UI listening on http://%s:%d",
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
            "parameter tuning web UI accept failed: %s", std::strerror(errno));
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
    } else if (method == "GET" && path == "/api/params") {
      send_response(client_fd, "200 OK", "application/json", parameters_json());
    } else if (method == "POST" && path == "/api/set") {
      send_response(client_fd, "200 OK", "application/json", set_parameter_json(query));
    } else {
      send_response(client_fd, "404 Not Found", "text/plain", "not found\n");
    }
    ::close(client_fd);
  }

  static const std::vector<TunableParameter> & specs()
  {
    static const std::vector<TunableParameter> parameters = {
      {"segmentation", "/cv_pipe_segmenter", "min_red", TunableType::Integer, 0, 255},
      {"segmentation", "/cv_pipe_segmenter", "min_green", TunableType::Integer, 0, 255},
      {"segmentation", "/cv_pipe_segmenter", "min_blue", TunableType::Integer, 0, 255},
      {"segmentation", "/cv_pipe_segmenter", "max_channel_diff", TunableType::Integer, 0, 255},
      {"segmentation", "/cv_pipe_segmenter", "max_red_blue_diff", TunableType::Integer, -255, 255},
      {"segmentation", "/cv_pipe_segmenter", "min_area", TunableType::Integer, 0, 100000},
      {"segmentation", "/cv_pipe_segmenter", "min_height_ratio", TunableType::Double, 0.0, 1.0},
      {"segmentation", "/cv_pipe_segmenter", "min_aspect", TunableType::Double, 0.0, 10.0},
      {"segmentation", "/cv_pipe_segmenter", "max_aspect", TunableType::Double, 0.0, 20.0},
      {"segmentation", "/cv_pipe_segmenter", "erode_kernel_size", TunableType::Integer, 1, 31},
      {"control", "/pipe_tracker", "manual_control_enabled", TunableType::Bool, 0, 1},
      {"control", "/pipe_tracker", "min_pixels", TunableType::Integer, 0, 100000},
      {"control", "/pipe_tracker", "desired_angle_deg", TunableType::Double, -180.0, 180.0},
      {"control", "/pipe_tracker", "forward_axis", TunableType::Integer, -1000, 1000},
      {"control", "/pipe_tracker", "z_axis", TunableType::Integer, 0, 1000},
      {"control", "/pipe_tracker", "lateral_gain", TunableType::Double, 0.0, 1000.0},
      {"control", "/pipe_tracker", "yaw_gain", TunableType::Double, 0.0, 1000.0},
      {"control", "/pipe_tracker", "lateral_sign", TunableType::Double, -1.0, 1.0},
      {"control", "/pipe_tracker", "yaw_sign", TunableType::Double, -1.0, 1.0},
      {"control", "/pipe_tracker", "command_rate_hz", TunableType::Double, 1.0, 30.0},
      {"control", "/pipe_tracker", "stale_timeout_s", TunableType::Double, 0.0, 5.0},
      {"control", "/pipe_tracker", "manual_override_timeout_s", TunableType::Double, 0.0, 5.0},
    };
    return parameters;
  }

  const TunableParameter * find_spec(const std::string & group, const std::string & name) const
  {
    for (const auto & spec : specs()) {
      if (group == spec.group && name == spec.name) {
        return &spec;
      }
    }
    return nullptr;
  }

  std::string parameters_json()
  {
    std::ostringstream out;
    out << "{\"ok\":true,\"segmentation\":"
        << get_group_json("segmentation", cv_get_client_)
        << ",\"control\":"
        << get_group_json("control", tracker_get_client_)
        << "}";
    return out.str();
  }

  std::string get_group_json(
    const std::string & group,
    const rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr & client)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(300))) {
      return "{\"ok\":false,\"message\":\"service unavailable\",\"values\":{}}";
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    for (const auto & spec : specs()) {
      if (group == spec.group) {
        request->names.emplace_back(spec.name);
      }
    }

    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      return "{\"ok\":false,\"message\":\"service timeout\",\"values\":{}}";
    }

    const auto response = future.get();
    std::ostringstream out;
    out << "{\"ok\":true,\"message\":\"ok\",\"values\":{";
    for (size_t i = 0; i < request->names.size(); ++i) {
      if (i != 0) {
        out << ",";
      }
      out << "\"" << json_escape(request->names[i]) << "\":";
      if (i < response->values.size()) {
        out << parameter_value_json(response->values[i]);
      } else {
        out << "null";
      }
    }
    out << "}}";
    return out.str();
  }

  std::string set_parameter_json(const std::string & query)
  {
    const std::string group = query_string(query, "group", "");
    const std::string name = query_string(query, "name", "");
    const std::string value_text = query_string(query, "value", "");
    const auto * spec = find_spec(group, name);
    if (spec == nullptr) {
      return make_json(false, "unknown parameter");
    }

    auto client = group == "segmentation" ? cv_set_client_ : tracker_set_client_;
    if (!client->wait_for_service(std::chrono::milliseconds(300))) {
      return make_json(false, "service unavailable");
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    try {
      if (spec->type == TunableType::Bool) {
        request->parameters.push_back(
          rclcpp::Parameter(spec->name, parse_bool(value_text)).to_parameter_msg());
      } else if (spec->type == TunableType::Integer) {
        const long value = std::lround(std::stod(value_text));
        if (value < spec->min_value || value > spec->max_value) {
          return make_json(false, "value out of range");
        }
        request->parameters.push_back(
          rclcpp::Parameter(spec->name, static_cast<int64_t>(value)).to_parameter_msg());
      } else {
        const double value = std::stod(value_text);
        if (value < spec->min_value || value > spec->max_value) {
          return make_json(false, "value out of range");
        }
        request->parameters.push_back(rclcpp::Parameter(spec->name, value).to_parameter_msg());
      }
    } catch (const std::exception & error) {
      return make_json(false, error.what());
    }

    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      return make_json(false, "service timeout");
    }

    const auto response = future.get();
    if (response->results.empty()) {
      return make_json(false, "empty parameter response");
    }
    if (!response->results.front().successful) {
      return make_json(false, response->results.front().reason);
    }
    return make_json(true, group + "." + name + "=" + value_text);
  }

  static std::string parameter_value_json(const rcl_interfaces::msg::ParameterValue & value)
  {
    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      return value.bool_value ? "true" : "false";
    }
    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      return std::to_string(value.integer_value);
    }
    if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      std::ostringstream out;
      out << value.double_value;
      return out.str();
    }
    return "null";
  }

  static bool parse_bool(const std::string & value)
  {
    return value == "1" || value == "true" || value == "on" || value == "yes";
  }

  static std::string query_string(
    const std::string & query, const std::string & key, const std::string & default_value)
  {
    size_t pos = 0;
    while (pos < query.size()) {
      const size_t next = query.find('&', pos);
      const std::string item = query.substr(pos, next == std::string::npos ? next : next - pos);
      const size_t eq = item.find('=');
      if (eq != std::string::npos && url_decode(item.substr(0, eq)) == key) {
        return url_decode(item.substr(eq + 1));
      }
      if (next == std::string::npos) {
        break;
      }
      pos = next + 1;
    }
    return default_value;
  }

  static std::string url_decode(const std::string & input)
  {
    std::string decoded;
    decoded.reserve(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
      if (input[i] == '%' && i + 2 < input.size()) {
        const auto hex = input.substr(i + 1, 2);
        try {
          decoded.push_back(static_cast<char>(std::stoi(hex, nullptr, 16)));
          i += 2;
        } catch (const std::exception &) {
          decoded.push_back(input[i]);
        }
      } else if (input[i] == '+') {
        decoded.push_back(' ');
      } else {
        decoded.push_back(input[i]);
      }
    }
    return decoded;
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
<title>Pipe Parameter Tuning</title>
<style>
:root { color-scheme: dark; font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
* { box-sizing: border-box; }
body { margin: 0; background: #111416; color: #eef2f4; }
header { display: flex; align-items: center; justify-content: space-between; gap: 16px; padding: 14px 18px; border-bottom: 1px solid #2d3438; }
h1 { margin: 0; font-size: 20px; font-weight: 650; letter-spacing: 0; }
main { display: grid; grid-template-columns: repeat(2, minmax(320px, 1fr)); gap: 16px; padding: 16px; }
.panel { border: 1px solid #2d3438; border-radius: 8px; background: #181d20; overflow: hidden; }
.panel h2 { margin: 0; padding: 10px 12px; font-size: 14px; font-weight: 650; color: #c2ccd2; border-bottom: 1px solid #2d3438; }
.stack { display: grid; gap: 8px; padding: 12px; }
.row { display: grid; grid-template-columns: 180px minmax(120px, 1fr) 92px; gap: 10px; align-items: center; min-height: 34px; }
label { color: #dce4e8; font-size: 13px; }
input[type=range] { width: 100%; }
input[type=number] { width: 92px; height: 30px; border: 1px solid #3c464c; border-radius: 6px; background: #0d1012; color: #eef2f4; padding: 0 8px; }
input[type=checkbox] { width: 20px; height: 20px; accent-color: #1b8a67; }
button { min-height: 36px; border: 1px solid #3c464c; border-radius: 7px; background: #222a2f; color: #eef2f4; font-size: 14px; font-weight: 650; cursor: pointer; }
button:hover { background: #2c363c; }
button.primary { background: #176b54; border-color: #21906f; }
.toolbar { display: flex; gap: 10px; padding: 0 16px 16px; }
#status { min-height: 36px; margin: 0 16px 16px; padding: 10px 12px; border-radius: 7px; background: #0d1012; color: #c2ccd2; font: 12px/1.35 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
#health { color: #c2ccd2; font-size: 13px; }
@media (max-width: 820px) {
  main { grid-template-columns: 1fr; padding: 12px; }
  .row { grid-template-columns: 1fr; gap: 6px; padding-bottom: 8px; border-bottom: 1px solid #242b2f; }
  input[type=number] { width: 100%; }
}
</style>
</head>
<body>
<header><h1>Pipe Parameter Tuning</h1><span id="health">Connecting</span></header>
<main>
  <section class="panel"><h2>Segmentation</h2><div id="segmentation" class="stack"></div></section>
  <section class="panel"><h2>Control</h2><div id="control" class="stack"></div></section>
</main>
<div class="toolbar"><button class="primary" id="refresh">Refresh</button><button id="defaults">Launch Defaults</button></div>
<div id="status">Ready</div>
<script>
const specs = [
  { group: 'segmentation', name: 'min_red', label: 'min_red', type: 'int', min: 0, max: 255, step: 1, def: 150 },
  { group: 'segmentation', name: 'min_green', label: 'min_green', type: 'int', min: 0, max: 255, step: 1, def: 150 },
  { group: 'segmentation', name: 'min_blue', label: 'min_blue', type: 'int', min: 0, max: 255, step: 1, def: 150 },
  { group: 'segmentation', name: 'max_channel_diff', label: 'max_channel_diff', type: 'int', min: 0, max: 255, step: 1, def: 45 },
  { group: 'segmentation', name: 'max_red_blue_diff', label: 'max_red_blue_diff', type: 'int', min: -255, max: 255, step: 1, def: 55 },
  { group: 'segmentation', name: 'min_area', label: 'min_area', type: 'int', min: 0, max: 100000, step: 100, def: 3000 },
  { group: 'segmentation', name: 'min_height_ratio', label: 'min_height_ratio', type: 'double', min: 0, max: 1, step: 0.01, def: 0.18 },
  { group: 'segmentation', name: 'min_aspect', label: 'min_aspect', type: 'double', min: 0, max: 10, step: 0.05, def: 0.5 },
  { group: 'segmentation', name: 'max_aspect', label: 'max_aspect', type: 'double', min: 0, max: 20, step: 0.05, def: 2.0 },
  { group: 'segmentation', name: 'erode_kernel_size', label: 'erode_kernel_size', type: 'int', min: 1, max: 31, step: 1, def: 1 },
  { group: 'control', name: 'manual_control_enabled', label: 'manual_control_enabled', type: 'bool', def: true },
  { group: 'control', name: 'min_pixels', label: 'min_pixels', type: 'int', min: 0, max: 100000, step: 10, def: 80 },
  { group: 'control', name: 'desired_angle_deg', label: 'desired_angle_deg', type: 'double', min: -180, max: 180, step: 1, def: 90 },
  { group: 'control', name: 'forward_axis', label: 'forward_axis', type: 'int', min: -1000, max: 1000, step: 10, def: 150 },
  { group: 'control', name: 'z_axis', label: 'z_axis', type: 'int', min: 0, max: 1000, step: 10, def: 500 },
  { group: 'control', name: 'lateral_gain', label: 'lateral_gain', type: 'double', min: 0, max: 1000, step: 10, def: 450 },
  { group: 'control', name: 'yaw_gain', label: 'yaw_gain', type: 'double', min: 0, max: 1000, step: 10, def: 450 },
  { group: 'control', name: 'lateral_sign', label: 'lateral_sign', type: 'double', min: -1, max: 1, step: 2, def: 1 },
  { group: 'control', name: 'yaw_sign', label: 'yaw_sign', type: 'double', min: -1, max: 1, step: 2, def: 1 },
  { group: 'control', name: 'command_rate_hz', label: 'command_rate_hz', type: 'double', min: 1, max: 30, step: 1, def: 10 },
  { group: 'control', name: 'stale_timeout_s', label: 'stale_timeout_s', type: 'double', min: 0, max: 5, step: 0.05, def: 0.5 },
  { group: 'control', name: 'manual_override_timeout_s', label: 'manual_override_timeout_s', type: 'double', min: 0, max: 5, step: 0.05, def: 0.5 }
];
const statusEl = document.getElementById('status');
const healthEl = document.getElementById('health');
const controls = new Map();
let timers = new Map();

function rowId(spec) { return `${spec.group}.${spec.name}`; }
function formatValue(spec, value) {
  if (spec.type === 'bool') return Boolean(value);
  if (spec.type === 'int') return Math.round(Number(value));
  return Number(value);
}
function setStatus(text) { statusEl.textContent = text; }
function build() {
  for (const spec of specs) {
    const row = document.createElement('label');
    row.className = 'row';
    const name = document.createElement('span');
    name.textContent = spec.label;
    row.appendChild(name);
    if (spec.type === 'bool') {
      const box = document.createElement('input');
      box.type = 'checkbox';
      row.appendChild(box);
      row.appendChild(document.createElement('span'));
      box.addEventListener('change', () => apply(spec, box.checked ? 'true' : 'false'));
      controls.set(rowId(spec), { box });
    } else {
      const slider = document.createElement('input');
      slider.type = 'range';
      slider.min = spec.min;
      slider.max = spec.max;
      slider.step = spec.step;
      const number = document.createElement('input');
      number.type = 'number';
      number.min = spec.min;
      number.max = spec.max;
      number.step = spec.step;
      row.appendChild(slider);
      row.appendChild(number);
      const sync = (from, to) => { to.value = from.value; debounceApply(spec, from.value); };
      slider.addEventListener('input', () => sync(slider, number));
      number.addEventListener('change', () => sync(number, slider));
      controls.set(rowId(spec), { slider, number });
    }
    document.getElementById(spec.group).appendChild(row);
  }
}
function setControl(spec, value) {
  const control = controls.get(rowId(spec));
  if (!control) return;
  const next = formatValue(spec, value);
  if (spec.type === 'bool') {
    control.box.checked = next;
  } else {
    control.slider.value = next;
    control.number.value = next;
  }
}
function debounceApply(spec, value) {
  const id = rowId(spec);
  if (timers.has(id)) clearTimeout(timers.get(id));
  timers.set(id, setTimeout(() => apply(spec, value), 180));
}
async function apply(spec, value) {
  try {
    const url = `/api/set?group=${encodeURIComponent(spec.group)}&name=${encodeURIComponent(spec.name)}&value=${encodeURIComponent(value)}`;
    const response = await fetch(url, { method: 'POST' });
    const data = await response.json();
    setStatus(data.message || JSON.stringify(data));
    healthEl.textContent = data.ok ? 'Updated' : 'Rejected';
  } catch (error) {
    healthEl.textContent = 'Disconnected';
    setStatus(String(error));
  }
}
async function refresh() {
  try {
    const response = await fetch('/api/params');
    const data = await response.json();
    for (const spec of specs) {
      const group = data[spec.group];
      if (group && group.values && group.values[spec.name] !== null && group.values[spec.name] !== undefined) {
        setControl(spec, group.values[spec.name]);
      }
    }
    healthEl.textContent = `Seg ${data.segmentation.ok ? 'online' : 'offline'} / Ctrl ${data.control.ok ? 'online' : 'offline'}`;
    setStatus(JSON.stringify({ segmentation: data.segmentation.message, control: data.control.message }));
  } catch (error) {
    healthEl.textContent = 'Disconnected';
    setStatus(String(error));
  }
}
async function defaults() {
  for (const spec of specs) {
    setControl(spec, spec.def);
    await apply(spec, spec.def);
  }
  refresh();
}
document.getElementById('refresh').addEventListener('click', refresh);
document.getElementById('defaults').addEventListener('click', defaults);
build();
refresh();
setInterval(refresh, 3000);
</script>
</body>
</html>
)HTML";
  }

  std::string http_host_;
  int http_port_;
  std::string segmenter_node_;
  std::string tracker_node_;

  std::atomic<bool> running_{false};
  int listen_fd_ = -1;
  std::thread server_thread_;
  std::vector<std::thread> client_threads_;

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr cv_get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr cv_set_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr tracker_get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr tracker_set_client_;
};

}  // namespace rov_pipe_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(rov_pipe_tracker::ParameterTuningWebUiNode)
