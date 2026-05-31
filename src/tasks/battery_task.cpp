// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/battery_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace
{

std::string read_string_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + path.string());
  }

  std::string value;
  std::getline(file, value);
  return value;
}

double read_double_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + path.string());
  }

  double value = 0.0;
  if (!(file >> value)) {
    throw std::runtime_error("Failed to parse " + path.string());
  }
  return value;
}

int read_int_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + path.string());
  }

  int value = 0;
  if (!(file >> value)) {
    throw std::runtime_error("Failed to parse " + path.string());
  }
  return value;
}

std::string format_capacity_message(
  const std::string & supply,
  double capacity,
  const std::string & threshold_name,
  double threshold)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(1) << supply << " capacity " << capacity
          << "% below " << threshold_name << " " << threshold << "%";
  return message.str();
}

}  // namespace

namespace system_diagnostics::tasks
{

void BatteryTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  power_supply_path_ = declare_or_get<std::string>(
    node, parameter_namespace + ".power_supply_path", "/sys/class/power_supply");
  supplies_ = declare_or_get<std::vector<std::string>>(
    node, parameter_namespace + ".supplies", {"BAT0"});
  warn_capacity_ = declare_or_get(node, parameter_namespace + ".warn_capacity", 30.0);
  error_capacity_ = declare_or_get(node, parameter_namespace + ".error_capacity", 15.0);
  require_present_ = declare_or_get(node, parameter_namespace + ".require_present", true);
}

void BatteryTask::cleanup() {}

void BatteryTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Battery state OK";

  try {
    if (supplies_.empty()) {
      throw std::runtime_error("No battery supplies configured");
    }

    status.add("power_supply_path", power_supply_path_);
    for (const auto & supply : supplies_) {
      const auto supply_path = std::filesystem::path(power_supply_path_) / supply;
      if (!std::filesystem::exists(supply_path)) {
        throw std::runtime_error("Battery supply does not exist: " + supply_path.string());
      }

      const auto present_path = supply_path / "present";
      const int present = std::filesystem::exists(present_path) ? read_int_file(present_path) : 1;
      if (require_present_ && present == 0) {
        throw std::runtime_error(supply + " is not present");
      }

      const double capacity = read_double_file(supply_path / "capacity");
      const auto battery_status = read_string_file(supply_path / "status");
      status.add(supply + ".present", present);
      status.add(supply + ".capacity_percent", capacity);
      status.add(supply + ".status", battery_status);

      if (capacity <= error_capacity_) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = format_capacity_message(supply, capacity, "error_capacity", error_capacity_);
      } else if (
        capacity <= warn_capacity_ && level < diagnostic_msgs::msg::DiagnosticStatus::WARN)
      {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        message = format_capacity_message(supply, capacity, "warn_capacity", warn_capacity_);
      }
    }
  } catch (const std::exception & error) {
    status.add("power_supply_path", power_supply_path_);
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = error.what();
  }

  status.summary(level, message);
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::BatteryTask, system_diagnostics::DiagnosticTask)
