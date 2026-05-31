// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__DIAGNOSTIC_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__DIAGNOSTIC_TASK_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <string>

namespace system_diagnostics
{

/**
 * @brief Base interface for pluginlib diagnostic tasks.
 */
class DiagnosticTask
{
public:
  virtual ~DiagnosticTask() = default;

  /**
   * @brief Configure the task from parameters under the given namespace.
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) = 0;

  /**
   * @brief Release all task-owned resources and transient state.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Return the diagnostic status name shown on /diagnostics.
   */
  virtual std::string name() const {return diagnostic_name_;}

  /**
   * @brief Fill one diagnostic status update.
   */
  virtual void update(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;

protected:
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

  void configure_name(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace,
    const std::string & default_name)
  {
    diagnostic_name_ = declare_or_get<std::string>(
      node, parameter_namespace + ".name", default_name);
  }

private:
  std::string diagnostic_name_;
};

}  // namespace system_diagnostics

#endif  // SYSTEM_DIAGNOSTICS__DIAGNOSTIC_TASK_HPP_
