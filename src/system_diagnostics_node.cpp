// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/system_diagnostics_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <utility>

namespace
{

constexpr char kPackageName[] = "system_diagnostics";
constexpr char kBaseClassName[] = "system_diagnostics::DiagnosticTask";

bool is_task_parameter(const std::string & parameter_name, const std::string & task_name)
{
  return parameter_name.rfind(task_name + ".", 0) == 0;
}

}  // namespace

namespace system_diagnostics
{

SystemDiagnosticsNode::SystemDiagnosticsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("system_diagnostics", options),
  class_loader_(kPackageName, kBaseClassName)
{
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  declare_node_parameters();
  declare_task_parameters_from_overrides();
}

SystemDiagnosticsNode::~SystemDiagnosticsNode()
{
  cleanup_tasks();
}

SystemDiagnosticsNode::CallbackReturn SystemDiagnosticsNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  declare_node_parameters();
  update_rate_ = get_parameter("update_rate").as_double();
  if (update_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "update_rate must be positive; using 1.0 Hz");
    update_rate_ = 1.0;
  }

  hardware_id_ = get_parameter("hardware_id").as_string();
  load_tasks();
  RCLCPP_INFO(get_logger(), "Configured %zu diagnostic task(s)", tasks_.size());
  return CallbackReturn::SUCCESS;
}

SystemDiagnosticsNode::CallbackReturn SystemDiagnosticsNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  start_timer();
  return CallbackReturn::SUCCESS;
}

SystemDiagnosticsNode::CallbackReturn SystemDiagnosticsNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_timer();
  return CallbackReturn::SUCCESS;
}

SystemDiagnosticsNode::CallbackReturn SystemDiagnosticsNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_timer();
  cleanup_tasks();
  return CallbackReturn::SUCCESS;
}

SystemDiagnosticsNode::CallbackReturn SystemDiagnosticsNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stop_timer();
  cleanup_tasks();
  return CallbackReturn::SUCCESS;
}

SystemDiagnosticsNode::CallbackReturn SystemDiagnosticsNode::on_error(
  const rclcpp_lifecycle::State &)
{
  stop_timer();
  cleanup_tasks();
  return CallbackReturn::SUCCESS;
}

void SystemDiagnosticsNode::declare_node_parameters()
{
  if (!has_parameter("update_rate")) {
    declare_parameter<double>("update_rate", 1.0);
  }
  if (!has_parameter("hardware_id")) {
    declare_parameter<std::string>("hardware_id", "host");
  }
  if (!has_parameter("tasks")) {
    declare_parameter<std::vector<std::string>>("tasks", std::vector<std::string>{});
  }
  task_names_ = get_parameter("tasks").as_string_array();
}

void SystemDiagnosticsNode::declare_task_parameters_from_overrides()
{
  task_names_ = get_parameter("tasks").as_string_array();
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();

  for (const auto & task_name : task_names_) {
    for (const auto & [name, value] : overrides) {
      if (is_task_parameter(name, task_name) && !has_parameter(name)) {
        declare_parameter(name, value);
      }
    }
  }
}

void SystemDiagnosticsNode::load_tasks()
{
  cleanup_tasks();

  const auto plugins = discover_plugins();
  for (const auto & [parameter_namespace, plugin_name] : plugins) {
    if (!is_enabled(parameter_namespace)) {
      RCLCPP_INFO(
        get_logger(), "Skipping disabled diagnostic task namespace '%s'",
        parameter_namespace.c_str());
      continue;
    }

    try {
      auto task = class_loader_.createSharedInstance(plugin_name);
      task->configure(
        std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(shared_from_this()),
        parameter_namespace);
      tasks_.push_back(LoadedTask{parameter_namespace, plugin_name, task});
      RCLCPP_INFO(
        get_logger(), "Loaded diagnostic task '%s' from plugin '%s'",
        parameter_namespace.c_str(), plugin_name.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Failed to load diagnostic plugin '%s' for namespace '%s': %s",
        plugin_name.c_str(), parameter_namespace.c_str(), error.what());
    }
  }
}

void SystemDiagnosticsNode::cleanup_tasks()
{
  stop_timer();
  for (auto & loaded_task : tasks_) {
    try {
      loaded_task.task->cleanup();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Failed to clean up diagnostic task '%s': %s",
        loaded_task.parameter_namespace.c_str(), error.what());
    }
  }
  tasks_.clear();
}

void SystemDiagnosticsNode::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  for (const auto & loaded_task : tasks_) {
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = loaded_task.task->name();
    status.hardware_id = hardware_id_;

    try {
      loaded_task.task->update(status);
    } catch (const std::exception & error) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
    }

    array.status.push_back(status);
  }

  diagnostics_publisher_->publish(array);
}

void SystemDiagnosticsNode::start_timer()
{
  stop_timer();
  const auto period = std::chrono::duration<double>(1.0 / update_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {publish_diagnostics();});
}

void SystemDiagnosticsNode::stop_timer()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

bool SystemDiagnosticsNode::is_enabled(const std::string & parameter_namespace) const
{
  const auto parameter_name = parameter_namespace + ".enabled";
  if (!has_parameter(parameter_name)) {
    return true;
  }
  return get_parameter(parameter_name).as_bool();
}

std::map<std::string, std::string> SystemDiagnosticsNode::discover_plugins() const
{
  std::map<std::string, std::string> plugins;

  for (const auto & task_name : task_names_) {
    const auto plugin_parameter = task_name + ".plugin";
    if (has_parameter(plugin_parameter)) {
      plugins[task_name] = get_parameter(plugin_parameter).as_string();
    } else {
      RCLCPP_WARN(
        get_logger(), "Skipping diagnostic task namespace '%s' because '%s' is not declared",
        task_name.c_str(), plugin_parameter.c_str());
    }
  }

  return plugins;
}

}  // namespace system_diagnostics

RCLCPP_COMPONENTS_REGISTER_NODE(system_diagnostics::SystemDiagnosticsNode)
