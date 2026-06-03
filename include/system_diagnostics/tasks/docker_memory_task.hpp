// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#ifndef SYSTEM_DIAGNOSTICS__TASKS__DOCKER_MEMORY_TASK_HPP_
#define SYSTEM_DIAGNOSTICS__TASKS__DOCKER_MEMORY_TASK_HPP_

#include "system_diagnostics/diagnostic_task.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace system_diagnostics::tasks
{

/**
 * @brief Reports Docker container memory usage via the Docker Unix socket.
 */
class DockerMemoryTask : public DiagnosticTask
{
public:
  struct ContainerMemory
  {
    std::string id;
    std::string name;
    std::uint64_t usage_bytes = 0;
    std::uint64_t limit_bytes = 0;
  };

  struct Sample
  {
    bool success = false;
    std::string error;
    std::vector<ContainerMemory> containers;
  };

  using Sampler = std::function<Sample()>;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & parameter_namespace) override;
  void cleanup() override;
  void update(diagnostic_updater::DiagnosticStatusWrapper & status) override;
  void set_sampler(Sampler sampler);

private:
  Sample sample_docker() const;

  std::string socket_path_ = "/var/run/docker.sock";
  double warn_usage_ = 80.0;
  double error_usage_ = 90.0;
  std::uint64_t warn_usage_bytes_ = 0;
  std::uint64_t error_usage_bytes_ = 0;
  Sampler sampler_;
};

}  // namespace system_diagnostics::tasks

#endif  // SYSTEM_DIAGNOSTICS__TASKS__DOCKER_MEMORY_TASK_HPP_
