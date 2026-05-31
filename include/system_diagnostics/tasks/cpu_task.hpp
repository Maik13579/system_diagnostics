// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__CPU_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__CPU_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports aggregate CPU usage and load average diagnostics.
 */
class CpuTask : public DiagnosticTask
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  struct CpuSample
  {
    std::uint64_t idle = 0;
    std::uint64_t total = 0;
  };

  static CpuSample read_cpu_sample(const std::string & proc_path);
  static bool read_load_average(
    const std::string & proc_path,
    double & load_1min,
    double & load_5min,
    double & load_15min);

  double warn_usage_ = 80.0;
  double error_usage_ = 95.0;
  double warn_load_per_core_ = 1.0;
  double error_load_per_core_ = 1.5;
  std::string proc_path_ = "/proc";
  std::optional<CpuSample> previous_sample_;
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__CPU_TASK_HPP_
