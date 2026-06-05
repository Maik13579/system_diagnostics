// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__NETWORK_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__NETWORK_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports Linux network interface carrier and counter deltas from sysfs.
 */
class NetworkTask : public DiagnosticTask
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  struct Counters
  {
    std::uint64_t rx_errors = 0;
    std::uint64_t tx_errors = 0;
    std::uint64_t rx_dropped = 0;
    std::uint64_t tx_dropped = 0;
  };

  struct Thresholds
  {
    std::uint64_t warn_rx_errors_delta = 1;
    std::uint64_t error_rx_errors_delta = 10;
    std::uint64_t warn_tx_errors_delta = 1;
    std::uint64_t error_tx_errors_delta = 10;
    std::uint64_t warn_rx_dropped_delta = 1;
    std::uint64_t error_rx_dropped_delta = 10;
    std::uint64_t warn_tx_dropped_delta = 1;
    std::uint64_t error_tx_dropped_delta = 10;
  };

  static Counters read_counters(const std::string & interface_path);

  std::string sysfs_path_ = "/sys/class/net";
  std::vector<std::string> interfaces_{"eth0"};
  bool require_carrier_ = true;
  Thresholds thresholds_;
  std::map<std::string, Counters> previous_counters_;
  std::uint64_t sample_count_ = 0;
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__NETWORK_TASK_HPP_
