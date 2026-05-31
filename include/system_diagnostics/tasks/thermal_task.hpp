// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__THERMAL_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__THERMAL_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <string>
#include <vector>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports Linux thermal zone temperatures from sysfs.
 */
class ThermalTask : public DiagnosticTask
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  std::string sysfs_path_ = "/sys/class/thermal";
  std::vector<std::string> zones_;
  double warn_temperature_c_ = 75.0;
  double error_temperature_c_ = 90.0;
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__THERMAL_TASK_HPP_
