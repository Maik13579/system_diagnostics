// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/storage_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <fcntl.h>
#include <pluginlib/class_list_macros.hpp>
#include <sys/statvfs.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

std::string join_path(const std::string & path, const std::string & name)
{
  if (path.empty() || path.back() == '/') {
    return path + name;
  }
  return path + "/" + name;
}

std::string format_usage_message(
  const std::string & path,
  double used_percent,
  const std::string & threshold_name,
  double threshold)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(1) << path << " used " << used_percent
          << "% exceeded " << threshold_name << " " << threshold << "%";
  return message.str();
}

}  // namespace

namespace system_diagnostics::tasks
{

void StorageTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  paths_ = declare_or_get<std::vector<std::string>>(node, parameter_namespace + ".paths", {"/"});
  if (paths_.empty()) {
    paths_.push_back("/");
  }
  warn_usage_ = declare_or_get(node, parameter_namespace + ".warn_usage", 85.0);
  error_usage_ = declare_or_get(node, parameter_namespace + ".error_usage", 95.0);
  check_writable_ = declare_or_get(node, parameter_namespace + ".check_writable", false);
}

void StorageTask::cleanup() {}

void StorageTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Storage usage OK";

  for (const auto & path : paths_) {
    try {
      struct statvfs data {};
      if (statvfs(path.c_str(), &data) != 0) {
        throw std::runtime_error(std::string("statvfs failed: ") + std::strerror(errno));
      }

      const auto total_bytes = static_cast<unsigned long long>(data.f_blocks) * data.f_frsize;
      const auto available_bytes = static_cast<unsigned long long>(data.f_bavail) * data.f_frsize;
      const auto used_bytes = total_bytes > available_bytes ? total_bytes - available_bytes : 0;
      const double used_percent = total_bytes == 0 ?
        0.0 :
        static_cast<double>(used_bytes) / static_cast<double>(total_bytes) * 100.0;
      const bool writable = !check_writable_ || check_path_writable(path);

      status.add(path + ".total_bytes", total_bytes);
      status.add(path + ".available_bytes", available_bytes);
      status.add(path + ".used_percent", used_percent);
      status.add(path + ".writable", writable ? "true" : "false");

      if (check_writable_ && !writable) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = path + " is not writable";
      } else if (used_percent >= error_usage_) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = format_usage_message(path, used_percent, "error_usage", error_usage_);
      } else if (used_percent >= warn_usage_ &&
        level < diagnostic_msgs::msg::DiagnosticStatus::WARN)
      {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        message = format_usage_message(path, used_percent, "warn_usage", warn_usage_);
      }
    } catch (const std::exception & error) {
      status.add(path + ".error", error.what());
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = path + " check failed: " + error.what();
    }
  }

  status.summary(level, message);
}

bool StorageTask::check_path_writable(const std::string & path)
{
  const auto test_path = join_path(
    path, ".system_diagnostics_write_test." + std::to_string(getpid()));
  const int fd = open(test_path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0600);
  if (fd < 0) {
    return false;
  }
  close(fd);
  return unlink(test_path.c_str()) == 0;
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::StorageTask, system_diagnostics::DiagnosticTask)
