// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__SYSTEM_DIAGNOSTICS_NODE_HPP_
#define SYSTEM_DIAGNOSTICS__SYSTEM_DIAGNOSTICS_NODE_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace system_diagnostics
{

/**
 * @brief Lifecycle component that loads diagnostic task plugins from parameters.
 */
class SystemDiagnosticsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SystemDiagnosticsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SystemDiagnosticsNode() override;

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  struct LoadedTask
  {
    std::string parameter_namespace;
    std::string plugin_name;
    std::shared_ptr<DiagnosticTask> task;
  };

  void declare_node_parameters();
  void declare_task_parameters_from_overrides();
  void load_tasks();
  void cleanup_tasks();
  void start_timer();
  void stop_timer();
  bool is_enabled(const std::string & parameter_namespace) const;
  std::map<std::string, std::string> discover_plugins() const;

  diagnostic_updater::Updater updater_;
  pluginlib::ClassLoader<DiagnosticTask> class_loader_;
  std::vector<LoadedTask> tasks_;
  rclcpp::TimerBase::SharedPtr timer_;
  double update_rate_ = 1.0;
  std::vector<std::string> task_names_;
};

}  // namespace system_diagnostics

#endif  // SYSTEM_DIAGNOSTICS__SYSTEM_DIAGNOSTICS_NODE_HPP_
