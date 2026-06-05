// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/thermal_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
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

long read_long_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + path.string());
  }

  long value = 0;
  if (!(file >> value)) {
    throw std::runtime_error("Failed to parse " + path.string());
  }
  return value;
}

std::vector<std::string> discover_thermal_zones(const std::string & sysfs_path)
{
  std::vector<std::string> zones;
  if (!std::filesystem::exists(sysfs_path)) {
    throw std::runtime_error("Thermal sysfs path does not exist: " + sysfs_path);
  }

  for (const auto & entry : std::filesystem::directory_iterator(sysfs_path)) {
    if (!entry.is_directory()) {
      continue;
    }
    const auto name = entry.path().filename().string();
    if (name.rfind("thermal_zone", 0) == 0) {
      zones.push_back(name);
    }
  }
  std::sort(zones.begin(), zones.end());
  return zones;
}

std::string format_temperature_message(
  const std::string & zone_type,
  double temperature_c,
  const std::string & threshold_name,
  double threshold)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(1) << zone_type << " " << temperature_c
          << " C exceeded " << threshold_name << " " << threshold << " C";
  return message.str();
}

double elapsed_ms(
  const std::chrono::steady_clock::time_point & start,
  const std::chrono::steady_clock::time_point & stop)
{
  return std::chrono::duration<double, std::milli>(stop - start).count();
}

std::string format_ms(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

}  // namespace

namespace system_diagnostics::tasks
{

void ThermalTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  sysfs_path_ = declare_or_get<std::string>(
    node, parameter_namespace + ".sysfs_path", "/sys/class/thermal");
  zones_ = declare_or_get<std::vector<std::string>>(node, parameter_namespace + ".zones", {});
  warn_temperature_c_ = declare_or_get(
    node, parameter_namespace + ".warn_temperature_c", 75.0);
  error_temperature_c_ = declare_or_get(
    node, parameter_namespace + ".error_temperature_c", 90.0);
  discovery_error_.clear();
  try {
    active_zones_ = zones_.empty() ? discover_thermal_zones(sysfs_path_) : zones_;
  } catch (const std::exception & error) {
    active_zones_.clear();
    discovery_error_ = error.what();
  }
}

void ThermalTask::cleanup()
{
  active_zones_.clear();
  discovery_error_.clear();
}

void ThermalTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Thermal state OK";

  try {
    if (!discovery_error_.empty()) {
      throw std::runtime_error(discovery_error_);
    }
    if (active_zones_.empty()) {
      throw std::runtime_error("No thermal zones found in " + sysfs_path_);
    }

    status.add("sysfs_path", sysfs_path_);
    status.add("zone_count", static_cast<int>(active_zones_.size()));
    double worst_temperature_c = std::numeric_limits<double>::lowest();
    std::string worst_zone_type;
    for (const auto & zone : active_zones_) {
      const auto zone_start = std::chrono::steady_clock::now();
      const auto zone_path = std::filesystem::path(sysfs_path_) / zone;
      if (!std::filesystem::exists(zone_path)) {
        throw std::runtime_error("Thermal zone does not exist: " + zone_path.string());
      }

      const auto zone_type = read_string_file(zone_path / "type");
      const double temperature_c = static_cast<double>(read_long_file(zone_path / "temp")) / 1000.0;
      const auto label = zone + "." + zone_type;
      status.add(label + ".temperature_c", temperature_c);
      status.add(
        label + ".read_duration_ms",
        format_ms(elapsed_ms(zone_start, std::chrono::steady_clock::now())));

      if (temperature_c > worst_temperature_c) {
        worst_temperature_c = temperature_c;
        worst_zone_type = zone_type;
      }
    }

    status.add("worst_temperature_c", worst_temperature_c);
    status.add("worst_zone", worst_zone_type);
    if (worst_temperature_c >= error_temperature_c_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = format_temperature_message(
        worst_zone_type, worst_temperature_c, "error_temperature_c", error_temperature_c_);
    } else if (worst_temperature_c >= warn_temperature_c_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = format_temperature_message(
        worst_zone_type, worst_temperature_c, "warn_temperature_c", warn_temperature_c_);
    }
  } catch (const std::exception & error) {
    status.add("sysfs_path", sysfs_path_);
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = error.what();
  }

  status.summary(level, message);
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::ThermalTask, system_diagnostics::DiagnosticTask)
