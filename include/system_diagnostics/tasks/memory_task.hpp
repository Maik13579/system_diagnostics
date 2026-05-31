// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__MEMORY_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__MEMORY_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <cstdint>
#include <map>
#include <string>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports RAM and swap usage diagnostics from procfs.
 */
class MemoryTask : public DiagnosticTask
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  std::string name() const override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  static std::map<std::string, std::uint64_t> read_meminfo(const std::string & proc_path);

  std::string parameter_namespace_;
  double warn_usage_ = 85.0;
  double error_usage_ = 95.0;
  double warn_swap_usage_ = 20.0;
  double error_swap_usage_ = 50.0;
  std::string proc_path_ = "/proc";
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__MEMORY_TASK_HPP_
