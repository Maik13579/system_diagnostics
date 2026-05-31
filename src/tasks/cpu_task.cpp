// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/cpu_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

std::string format_threshold_message(
  const std::string & metric,
  double value,
  const std::string & threshold_name,
  double threshold,
  const std::string & suffix,
  int precision)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(precision) << metric << " " << value << suffix
          << " exceeded " << threshold_name << " " << threshold << suffix;
  return message.str();
}

}  // namespace

namespace system_diagnostics::tasks
{

void CpuTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  warn_usage_ = declare_or_get(node, parameter_namespace + ".warn_usage", 80.0);
  error_usage_ = declare_or_get(node, parameter_namespace + ".error_usage", 95.0);
  warn_load_per_core_ = declare_or_get(node, parameter_namespace + ".warn_load_per_core", 1.0);
  error_load_per_core_ = declare_or_get(node, parameter_namespace + ".error_load_per_core", 1.5);
  proc_path_ = declare_or_get<std::string>(node, parameter_namespace + ".proc_path", "/proc");
  previous_sample_.reset();
}

void CpuTask::cleanup()
{
  previous_sample_.reset();
}

void CpuTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  try {
    const auto sample = read_cpu_sample(proc_path_);
    double load_1min = 0.0;
    double load_5min = 0.0;
    double load_15min = 0.0;
    const bool have_load = read_load_average(proc_path_, load_1min, load_5min, load_15min);
    const auto concurrency = std::thread::hardware_concurrency();
    const auto num_cores = std::max(1u, concurrency);
    const double load_per_core = have_load ? load_1min / static_cast<double>(num_cores) : 0.0;

    status.add("load_1min", load_1min);
    status.add("load_5min", load_5min);
    status.add("load_15min", load_15min);
    status.add("load_per_core", load_per_core);
    status.add("num_cores", num_cores);
    status.add("proc_path", proc_path_);

    if (!previous_sample_) {
      previous_sample_ = sample;
      status.add("usage_percent", 0.0);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for second CPU sample");
      return;
    }

    const auto total_delta = sample.total - previous_sample_->total;
    const auto idle_delta = sample.idle - previous_sample_->idle;
    previous_sample_ = sample;

    const double usage_percent = total_delta == 0 ?
      0.0 :
      (1.0 - static_cast<double>(idle_delta) / static_cast<double>(total_delta)) * 100.0;
    status.add("usage_percent", usage_percent);

    if (usage_percent >= error_usage_) {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        format_threshold_message("CPU usage", usage_percent, "error_usage", error_usage_, "%", 1));
    } else if (load_per_core >= error_load_per_core_) {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        format_threshold_message(
          "CPU load/core", load_per_core, "error_load_per_core", error_load_per_core_, "", 2));
    } else if (usage_percent >= warn_usage_) {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        format_threshold_message("CPU usage", usage_percent, "warn_usage", warn_usage_, "%", 1));
    } else if (load_per_core >= warn_load_per_core_) {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        format_threshold_message(
          "CPU load/core", load_per_core, "warn_load_per_core", warn_load_per_core_, "", 2));
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "CPU usage OK");
    }
  } catch (const std::exception & error) {
    status.add("proc_path", proc_path_);
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
  }
}

CpuTask::CpuSample CpuTask::read_cpu_sample(const std::string & proc_path)
{
  std::ifstream file(proc_path + "/stat");
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + proc_path + "/stat");
  }

  std::string line;
  std::getline(file, line);
  std::istringstream stream(line);

  std::string label;
  CpuSample sample;
  std::uint64_t user = 0;
  std::uint64_t nice = 0;
  std::uint64_t system = 0;
  std::uint64_t idle = 0;
  std::uint64_t iowait = 0;
  std::uint64_t irq = 0;
  std::uint64_t softirq = 0;
  std::uint64_t steal = 0;
  std::uint64_t guest = 0;
  std::uint64_t guest_nice = 0;

  if (!(stream >> label) || label != "cpu") {
    throw std::runtime_error("Missing aggregate cpu line in " + proc_path + "/stat");
  }
  if (!(stream >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal)) {
    throw std::runtime_error("Failed to parse aggregate cpu line in " + proc_path + "/stat");
  }
  stream >> guest >> guest_nice;

  sample.idle = idle + iowait;
  sample.total = user + nice + system + idle + iowait + irq + softirq + steal + guest + guest_nice;
  return sample;
}

bool CpuTask::read_load_average(
  const std::string & proc_path,
  double & load_1min,
  double & load_5min,
  double & load_15min)
{
  std::ifstream file(proc_path + "/loadavg");
  if (!file.is_open()) {
    return false;
  }
  file >> load_1min >> load_5min >> load_15min;
  return static_cast<bool>(file);
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::CpuTask, system_diagnostics::DiagnosticTask)
