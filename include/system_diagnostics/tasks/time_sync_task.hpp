// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__TIME_SYNC_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__TIME_SYNC_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports Linux kernel time synchronization state using adjtimex.
 */
class TimeSyncTask : public DiagnosticTask
{
public:
  struct Sample
  {
    bool success = false;
    bool synchronized = false;
    double offset_ms = 0.0;
    double max_error_ms = 0.0;
    double estimated_error_ms = 0.0;
    int status_bits = 0;
    std::string error;
  };

  using Sampler = std::function<Sample()>;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;
  void set_sampler(Sampler sampler);

private:
  static Sample sample_adjtimex();

  bool require_synchronized_ = true;
  double warn_offset_ms_ = 50.0;
  double error_offset_ms_ = 100.0;
  double warn_max_error_ms_ = 1000.0;
  double error_max_error_ms_ = 5000.0;
  Sampler sampler_{sample_adjtimex};
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__TIME_SYNC_TASK_HPP_
