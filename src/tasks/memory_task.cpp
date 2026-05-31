// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/memory_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace
{

template<typename ValueT>
ValueT declare_or_get(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const ValueT & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<ValueT>(name, default_value);
  }
  return node->get_parameter(name).get_value<ValueT>();
}

}  // namespace

namespace system_diagnostics::tasks
{

void MemoryTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  parameter_namespace_ = parameter_namespace;
  warn_usage_ = declare_or_get(node, parameter_namespace + ".warn_usage", 85.0);
  error_usage_ = declare_or_get(node, parameter_namespace + ".error_usage", 95.0);
  warn_swap_usage_ = declare_or_get(node, parameter_namespace + ".warn_swap_usage", 20.0);
  error_swap_usage_ = declare_or_get(node, parameter_namespace + ".error_swap_usage", 50.0);
  proc_path_ = declare_or_get<std::string>(node, parameter_namespace + ".proc_path", "/proc");
}

void MemoryTask::cleanup() {}

std::string MemoryTask::name() const
{
  return parameter_namespace_.empty() ? "memory" : parameter_namespace_;
}

void MemoryTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  try {
    const auto meminfo = read_meminfo(proc_path_);
    const auto mem_total = meminfo.at("MemTotal");
    const auto mem_available = meminfo.at("MemAvailable");
    const auto swap_total = meminfo.at("SwapTotal");
    const auto swap_free = meminfo.at("SwapFree");

    if (mem_total == 0) {
      throw std::runtime_error("MemTotal is zero in " + proc_path_ + "/meminfo");
    }

    const double memory_used_percent =
      static_cast<double>(mem_total - mem_available) / static_cast<double>(mem_total) * 100.0;
    const double swap_used_percent = swap_total == 0 ?
      0.0 :
      static_cast<double>(swap_total - swap_free) / static_cast<double>(swap_total) * 100.0;

    status.add("memory_total_kb", mem_total);
    status.add("memory_available_kb", mem_available);
    status.add("memory_used_percent", memory_used_percent);
    status.add("swap_total_kb", swap_total);
    status.add("swap_free_kb", swap_free);
    status.add("swap_used_percent", swap_used_percent);
    status.add("proc_path", proc_path_);

    if (memory_used_percent >= error_usage_ || swap_used_percent >= error_swap_usage_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Memory threshold exceeded");
    } else if (memory_used_percent >= warn_usage_ || swap_used_percent >= warn_swap_usage_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Memory threshold warning");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Memory usage OK");
    }
  } catch (const std::exception & error) {
    status.add("proc_path", proc_path_);
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
  }
}

std::map<std::string, std::uint64_t> MemoryTask::read_meminfo(const std::string & proc_path)
{
  std::ifstream file(proc_path + "/meminfo");
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + proc_path + "/meminfo");
  }

  std::map<std::string, std::uint64_t> values;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    std::string key;
    std::uint64_t value = 0;
    std::string unit;
    if (stream >> key >> value >> unit) {
      if (!key.empty() && key.back() == ':') {
        key.pop_back();
      }
      values[key] = value;
    }
  }

  for (const auto * key : {"MemTotal", "MemAvailable", "SwapTotal", "SwapFree"}) {
    if (values.find(key) == values.end()) {
      throw std::runtime_error(std::string("Missing ") + key + " in " + proc_path + "/meminfo");
    }
  }
  return values;
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::MemoryTask, system_diagnostics::DiagnosticTask)
