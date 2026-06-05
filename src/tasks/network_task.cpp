// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/tasks/network_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

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

std::uint64_t read_uint64_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to read " + path.string());
  }

  std::uint64_t value = 0;
  if (!(file >> value)) {
    throw std::runtime_error("Failed to parse " + path.string());
  }
  return value;
}

std::uint64_t counter_delta(std::uint64_t current, std::uint64_t previous)
{
  if (current < previous) {
    return 0;
  }
  return current - previous;
}

void apply_counter_threshold(
  const std::string & interface,
  const std::string & counter,
  std::uint64_t delta,
  std::uint64_t warn_threshold,
  std::uint64_t error_threshold,
  int & level,
  std::string & message)
{
  if (delta >= error_threshold) {
    std::ostringstream stream;
    stream << interface << " " << counter << " delta " << delta
           << " exceeded error_" << counter << "_delta " << error_threshold;
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = stream.str();
  } else if (delta >= warn_threshold && level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    std::ostringstream stream;
    stream << interface << " " << counter << " delta " << delta
           << " exceeded warn_" << counter << "_delta " << warn_threshold;
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = stream.str();
  }
}

}  // namespace

namespace system_diagnostics::tasks
{

void NetworkTask::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_namespace)
{
  configure_name(node, parameter_namespace, "system_diagnostics/" + parameter_namespace);
  sysfs_path_ = declare_or_get<std::string>(
    node, parameter_namespace + ".sysfs_path", "/sys/class/net");
  interfaces_ = declare_or_get<std::vector<std::string>>(
    node, parameter_namespace + ".interfaces", {"eth0"});
  require_carrier_ = declare_or_get(node, parameter_namespace + ".require_carrier", true);
  thresholds_.warn_rx_errors_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".warn_rx_errors_delta", 1);
  thresholds_.error_rx_errors_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".error_rx_errors_delta", 10);
  thresholds_.warn_tx_errors_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".warn_tx_errors_delta", 1);
  thresholds_.error_tx_errors_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".error_tx_errors_delta", 10);
  thresholds_.warn_rx_dropped_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".warn_rx_dropped_delta", 1);
  thresholds_.error_rx_dropped_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".error_rx_dropped_delta", 10);
  thresholds_.warn_tx_dropped_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".warn_tx_dropped_delta", 1);
  thresholds_.error_tx_dropped_delta = declare_or_get<std::int64_t>(
    node, parameter_namespace + ".error_tx_dropped_delta", 10);
  previous_counters_.clear();
  sample_count_ = 0;
}

void NetworkTask::cleanup()
{
  previous_counters_.clear();
  sample_count_ = 0;
}

void NetworkTask::update(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Network state OK";
  ++sample_count_;
  status.add("sample_count", sample_count_);

  try {
    if (interfaces_.empty()) {
      throw std::runtime_error("No network interfaces configured");
    }

    status.add("sysfs_path", sysfs_path_);
    status.add("interface_count", static_cast<int>(interfaces_.size()));
    for (const auto & interface : interfaces_) {
      try {
        const auto interface_path = std::filesystem::path(sysfs_path_) / interface;
        if (!std::filesystem::exists(interface_path)) {
          throw std::runtime_error(
            "Network interface does not exist: " + interface_path.string());
        }

        const auto operstate = read_string_file(interface_path / "operstate");
        const auto carrier = read_string_file(interface_path / "carrier");
        const auto counters = read_counters(interface_path.string());
        status.add(interface + ".operstate", operstate);
        status.add(interface + ".carrier", carrier);
        status.add(interface + ".rx_errors", counters.rx_errors);
        status.add(interface + ".tx_errors", counters.tx_errors);
        status.add(interface + ".rx_dropped", counters.rx_dropped);
        status.add(interface + ".tx_dropped", counters.tx_dropped);

        if (require_carrier_ && carrier != "1") {
          level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          message = interface + " carrier is down";
        }

        const auto previous = previous_counters_.find(interface);
        if (previous != previous_counters_.end()) {
          const auto rx_errors_delta = counter_delta(
            counters.rx_errors, previous->second.rx_errors);
          const auto tx_errors_delta = counter_delta(
            counters.tx_errors, previous->second.tx_errors);
          const auto rx_dropped_delta = counter_delta(
            counters.rx_dropped, previous->second.rx_dropped);
          const auto tx_dropped_delta = counter_delta(
            counters.tx_dropped, previous->second.tx_dropped);

          status.add(interface + ".rx_errors_delta", rx_errors_delta);
          status.add(interface + ".tx_errors_delta", tx_errors_delta);
          status.add(interface + ".rx_dropped_delta", rx_dropped_delta);
          status.add(interface + ".tx_dropped_delta", tx_dropped_delta);

          apply_counter_threshold(
            interface, "rx_errors", rx_errors_delta,
            thresholds_.warn_rx_errors_delta, thresholds_.error_rx_errors_delta, level, message);
          apply_counter_threshold(
            interface, "tx_errors", tx_errors_delta,
            thresholds_.warn_tx_errors_delta, thresholds_.error_tx_errors_delta, level, message);
          apply_counter_threshold(
            interface, "rx_dropped", rx_dropped_delta,
            thresholds_.warn_rx_dropped_delta, thresholds_.error_rx_dropped_delta, level, message);
          apply_counter_threshold(
            interface, "tx_dropped", tx_dropped_delta,
            thresholds_.warn_tx_dropped_delta, thresholds_.error_tx_dropped_delta, level, message);
        }

        previous_counters_[interface] = counters;
      } catch (const std::exception & error) {
        status.add(interface + ".error", error.what());
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = error.what();
      }
    }
  } catch (const std::exception & error) {
    status.add("sysfs_path", sysfs_path_);
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = error.what();
  }

  status.summary(level, message);
}

NetworkTask::Counters NetworkTask::read_counters(const std::string & interface_path)
{
  const auto statistics_path = std::filesystem::path(interface_path) / "statistics";
  Counters counters;
  counters.rx_errors = read_uint64_file(statistics_path / "rx_errors");
  counters.tx_errors = read_uint64_file(statistics_path / "tx_errors");
  counters.rx_dropped = read_uint64_file(statistics_path / "rx_dropped");
  counters.tx_dropped = read_uint64_file(statistics_path / "tx_dropped");
  return counters;
}

}  // namespace system_diagnostics::tasks

PLUGINLIB_EXPORT_CLASS(system_diagnostics::tasks::NetworkTask, system_diagnostics::DiagnosticTask)
