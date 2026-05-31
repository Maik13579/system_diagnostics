// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/time_sync_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sys/timex.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{

std::string format_threshold_message(
  const std::string & metric,
  double value,
  const std::string & threshold_name,
  double threshold)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(3) << metric << " " << value
          << " ms exceeded " << threshold_name << " " << threshold << " ms";
  return message.str();
}

}  // namespace

namespace system_diagnostics::tasks
{

void TimeSyncTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  require_synchronized_ = declare_or_get(
    node, parameter_namespace + ".require_synchronized", true);
  warn_offset_ms_ = declare_or_get(node, parameter_namespace + ".warn_offset_ms", 50.0);
  error_offset_ms_ = declare_or_get(node, parameter_namespace + ".error_offset_ms", 100.0);
  warn_max_error_ms_ = declare_or_get(node, parameter_namespace + ".warn_max_error_ms", 1000.0);
  error_max_error_ms_ = declare_or_get(node, parameter_namespace + ".error_max_error_ms", 5000.0);
}

void TimeSyncTask::cleanup() {}

void TimeSyncTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto sample = sampler_();
  status.add("offset_ms", sample.offset_ms);
  status.add("max_error_ms", sample.max_error_ms);
  status.add("estimated_error_ms", sample.estimated_error_ms);
  status.add("status_bits", sample.status_bits);
  status.add("synchronized", sample.synchronized ? "true" : "false");

  if (!sample.success) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, sample.error);
    return;
  }

  const auto absolute_offset_ms = std::abs(sample.offset_ms);
  if (require_synchronized_ && !sample.synchronized) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Time synchronization is unsynchronized");
  } else if (absolute_offset_ms >= error_offset_ms_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      format_threshold_message("Time offset", absolute_offset_ms, "error_offset_ms", error_offset_ms_));
  } else if (sample.max_error_ms >= error_max_error_ms_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      format_threshold_message(
        "Time max error", sample.max_error_ms, "error_max_error_ms", error_max_error_ms_));
  } else if (absolute_offset_ms >= warn_offset_ms_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      format_threshold_message("Time offset", absolute_offset_ms, "warn_offset_ms", warn_offset_ms_));
  } else if (sample.max_error_ms >= warn_max_error_ms_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      format_threshold_message(
        "Time max error", sample.max_error_ms, "warn_max_error_ms", warn_max_error_ms_));
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Time synchronization OK");
  }
}

void TimeSyncTask::set_sampler(Sampler sampler)
{
  sampler_ = std::move(sampler);
}

TimeSyncTask::Sample TimeSyncTask::sample_adjtimex()
{
  struct timex data {};
  errno = 0;
  const int result = adjtimex(&data);
  Sample sample;
  if (result < 0) {
    sample.success = false;
    sample.error = std::string("adjtimex failed: ") + std::strerror(errno);
    return sample;
  }

  sample.success = true;
  sample.status_bits = data.status;
  sample.synchronized = (data.status & STA_UNSYNC) == 0 && result != TIME_ERROR;
  sample.offset_ms = static_cast<double>(data.offset) / 1000.0;
  sample.max_error_ms = static_cast<double>(data.maxerror) / 1000.0;
  sample.estimated_error_ms = static_cast<double>(data.esterror) / 1000.0;
  return sample;
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::TimeSyncTask, system_diagnostics::DiagnosticTask)
