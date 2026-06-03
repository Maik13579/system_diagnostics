// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/docker_memory_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{

std::string format_percent_message(
  const std::string & container,
  double value,
  const std::string & threshold_name,
  double threshold)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(1) << container << " memory usage " << value
          << "% exceeded " << threshold_name << " " << threshold << "%";
  return message.str();
}

std::string format_bytes_message(
  const std::string & container,
  std::uint64_t value,
  const std::string & threshold_name,
  std::uint64_t threshold)
{
  std::ostringstream message;
  message << container << " memory usage " << value << " B exceeded " << threshold_name
          << " " << threshold << " B";
  return message.str();
}

std::string format_memory_usage(std::uint64_t bytes)
{
  constexpr double kib = 1024.0;
  constexpr double mib = kib * 1024.0;
  constexpr double gib = mib * 1024.0;

  std::ostringstream value;
  value << std::fixed << std::setprecision(1);
  if (bytes >= static_cast<std::uint64_t>(gib)) {
    value << static_cast<double>(bytes) / gib << " GiB";
  } else if (bytes >= static_cast<std::uint64_t>(mib)) {
    value << static_cast<double>(bytes) / mib << " MiB";
  } else if (bytes >= static_cast<std::uint64_t>(kib)) {
    value << static_cast<double>(bytes) / kib << " KiB";
  } else {
    value << bytes << " B";
  }
  return value.str();
}

std::string http_get_unix_socket(const std::string & socket_path, const std::string & path)
{
  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
  }

  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  if (socket_path.size() >= sizeof(address.sun_path)) {
    ::close(fd);
    throw std::runtime_error("Docker socket path is too long: " + socket_path);
  }
  std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1);

  if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    const auto error = std::string("connect failed for ") + socket_path + ": " + std::strerror(errno);
    ::close(fd);
    throw std::runtime_error(error);
  }

  const std::string request =
    "GET " + path + " HTTP/1.1\r\n"
    "Host: docker\r\n"
    "Connection: close\r\n\r\n";
  if (::send(fd, request.data(), request.size(), 0) < 0) {
    const auto error = std::string("send failed: ") + std::strerror(errno);
    ::close(fd);
    throw std::runtime_error(error);
  }

  std::string response;
  char buffer[4096];
  while (true) {
    const auto count = ::recv(fd, buffer, sizeof(buffer), 0);
    if (count < 0) {
      const auto error = std::string("recv failed: ") + std::strerror(errno);
      ::close(fd);
      throw std::runtime_error(error);
    }
    if (count == 0) {
      break;
    }
    response.append(buffer, static_cast<std::size_t>(count));
  }
  ::close(fd);

  const auto header_end = response.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    throw std::runtime_error("Docker API returned an invalid HTTP response");
  }

  const auto status_line_end = response.find("\r\n");
  const auto status_line = response.substr(0, status_line_end);
  if (status_line.find(" 200 ") == std::string::npos) {
    throw std::runtime_error("Docker API request failed: " + status_line);
  }

  return response.substr(header_end + 4);
}

std::vector<std::string> split_json_objects(const std::string & array)
{
  std::vector<std::string> objects;
  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  std::size_t object_start = std::string::npos;

  for (std::size_t i = 0; i < array.size(); ++i) {
    const char ch = array[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (ch == '\\' && in_string) {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (ch == '{') {
      if (depth == 0) {
        object_start = i;
      }
      ++depth;
    } else if (ch == '}') {
      --depth;
      if (depth == 0 && object_start != std::string::npos) {
        objects.push_back(array.substr(object_start, i - object_start + 1));
        object_start = std::string::npos;
      }
    }
  }
  return objects;
}

std::string extract_string_field(const std::string & json, const std::string & field)
{
  const auto key = "\"" + field + "\":\"";
  const auto start = json.find(key);
  if (start == std::string::npos) {
    return "";
  }
  const auto value_start = start + key.size();
  const auto value_end = json.find('"', value_start);
  if (value_end == std::string::npos) {
    return "";
  }
  return json.substr(value_start, value_end - value_start);
}

std::string extract_first_name(const std::string & json)
{
  const std::string key = "\"Names\":[\"";
  const auto start = json.find(key);
  if (start == std::string::npos) {
    return "";
  }
  const auto value_start = start + key.size();
  const auto value_end = json.find('"', value_start);
  if (value_end == std::string::npos) {
    return "";
  }
  auto name = json.substr(value_start, value_end - value_start);
  if (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  return name;
}

std::uint64_t extract_uint_field(const std::string & json, const std::string & field)
{
  const auto key = "\"" + field + "\":";
  const auto start = json.find(key);
  if (start == std::string::npos) {
    return 0;
  }
  const auto value_start = start + key.size();
  const auto value_end = json.find_first_not_of("0123456789", value_start);
  const auto value = json.substr(value_start, value_end - value_start);
  if (value.empty()) {
    return 0;
  }
  return static_cast<std::uint64_t>(std::stoull(value));
}

double usage_percent(const system_diagnostics::tasks::DockerMemoryTask::ContainerMemory & container)
{
  if (container.limit_bytes == 0) {
    return 0.0;
  }
  return static_cast<double>(container.usage_bytes) / static_cast<double>(container.limit_bytes) * 100.0;
}

}  // namespace

namespace system_diagnostics::tasks
{

void DockerMemoryTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  socket_path_ = declare_or_get<std::string>(
    node, parameter_namespace + ".socket_path", "/var/run/docker.sock");
  warn_usage_ = declare_or_get(node, parameter_namespace + ".warn_usage", 80.0);
  error_usage_ = declare_or_get(node, parameter_namespace + ".error_usage", 90.0);
  warn_usage_bytes_ = static_cast<std::uint64_t>(
    declare_or_get(node, parameter_namespace + ".warn_usage_bytes", 0));
  error_usage_bytes_ = static_cast<std::uint64_t>(
    declare_or_get(node, parameter_namespace + ".error_usage_bytes", 0));
  sampler_ = [this]() {return sample_docker();};
}

void DockerMemoryTask::cleanup() {}

void DockerMemoryTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto sample = sampler_();
  status.add("socket_path", socket_path_);
  status.add("container_count", static_cast<int>(sample.containers.size()));

  if (!sample.success) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, sample.error);
    return;
  }

  if (sample.containers.empty()) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No Docker containers found");
    return;
  }

  const auto worst = std::max_element(
    sample.containers.begin(), sample.containers.end(),
    [](const auto & left, const auto & right) {
      return usage_percent(left) < usage_percent(right);
    });

  for (const auto & container : sample.containers) {
    const auto prefix = "container." + container.name + ".";
    status.add(container.name, format_memory_usage(container.usage_bytes));
    status.add(prefix + "id", container.id);
    status.add(prefix + "usage_bytes", container.usage_bytes);
    status.add(prefix + "limit_bytes", container.limit_bytes);
    status.add(prefix + "usage_percent", usage_percent(container));
  }

  const auto worst_percent = usage_percent(*worst);
  status.add("worst_container", worst->name);
  status.add("worst_usage_bytes", worst->usage_bytes);
  status.add("worst_limit_bytes", worst->limit_bytes);
  status.add("worst_usage_percent", worst_percent);

  if (error_usage_bytes_ > 0 && worst->usage_bytes >= error_usage_bytes_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      format_bytes_message(
        worst->name, worst->usage_bytes, "error_usage_bytes", error_usage_bytes_));
  } else if (worst_percent >= error_usage_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      format_percent_message(worst->name, worst_percent, "error_usage", error_usage_));
  } else if (warn_usage_bytes_ > 0 && worst->usage_bytes >= warn_usage_bytes_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      format_bytes_message(worst->name, worst->usage_bytes, "warn_usage_bytes", warn_usage_bytes_));
  } else if (worst_percent >= warn_usage_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      format_percent_message(worst->name, worst_percent, "warn_usage", warn_usage_));
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Docker container memory usage OK");
  }
}

void DockerMemoryTask::set_sampler(Sampler sampler)
{
  sampler_ = std::move(sampler);
}

DockerMemoryTask::Sample DockerMemoryTask::sample_docker() const
{
  Sample sample;
  try {
    const auto containers_body = http_get_unix_socket(socket_path_, "/containers/json");
    for (const auto & object : split_json_objects(containers_body)) {
      ContainerMemory container;
      container.id = extract_string_field(object, "Id");
      container.name = extract_first_name(object);
      if (container.id.empty()) {
        continue;
      }
      if (container.name.empty()) {
        container.name = container.id.substr(0, std::min<std::size_t>(container.id.size(), 12));
      }

      const auto stats_body = http_get_unix_socket(
        socket_path_, "/containers/" + container.id + "/stats?stream=false");
      const auto memory_stats_start = stats_body.find("\"memory_stats\"");
      const auto memory_stats = memory_stats_start == std::string::npos ?
        stats_body :
        stats_body.substr(memory_stats_start);
      container.usage_bytes = extract_uint_field(memory_stats, "usage");
      container.limit_bytes = extract_uint_field(memory_stats, "limit");
      sample.containers.push_back(container);
    }
    sample.success = true;
  } catch (const std::exception & error) {
    sample.success = false;
    sample.error = error.what();
  }
  return sample;
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::DockerMemoryTask, system_diagnostics::DiagnosticTask)
