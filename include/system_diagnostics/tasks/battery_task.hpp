// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__BATTERY_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__BATTERY_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <string>
#include <vector>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports battery capacity and presence from Linux power-supply sysfs.
 */
class BatteryTask : public DiagnosticTask
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  std::string power_supply_path_ = "/sys/class/power_supply";
  std::vector<std::string> supplies_{"BAT0"};
  double warn_capacity_ = 30.0;
  double error_capacity_ = 15.0;
  bool require_present_ = true;
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__BATTERY_TASK_HPP_
