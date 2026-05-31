// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__STORAGE_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__STORAGE_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <string>
#include <vector>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports filesystem capacity and optional writability diagnostics.
 */
class StorageTask : public DiagnosticTask
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  static bool check_path_writable(const std::string & path);

  std::vector<std::string> paths_{"/"};
  double warn_usage_ = 85.0;
  double error_usage_ = 95.0;
  bool check_writable_ = false;
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__STORAGE_TASK_HPP_
