// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/diagnostic_task.hpp"
#include "system_diagnostics/system_diagnostics_node.hpp"
#include "system_diagnostics/tasks/cpu_task.hpp"
#include "system_diagnostics/tasks/memory_task.hpp"
#include "system_diagnostics/tasks/storage_task.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace
{

using namespace std::chrono_literals;

class HelperTask : public system_diagnostics::DiagnosticTask
{
public:
  using DiagnosticTask::configure_name;
  using DiagnosticTask::declare_or_get;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &,
    const std::string &) override {}
  void cleanup() override {}
  void update(diagnostic_updater::DiagnosticStatusWrapper &) override {}
};

class TemporaryDirectory
{
public:
  explicit TemporaryDirectory(const std::string & name)
  : path_(std::filesystem::temp_directory_path() / name)
  {
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const {return path_;}

private:
  std::filesystem::path path_;
};

rclcpp_lifecycle::LifecycleNode::SharedPtr make_node(
  const std::string & name,
  const std::vector<rclcpp::Parameter> & parameter_overrides = {})
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameter_overrides);
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options);
}

void write_file(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream file(path);
  file << contents;
}

}  // namespace

TEST(DiagnosticTaskHelpers, DeclareOrGetDeclaresMissingParameter)
{
  auto node = make_node("declare_missing_parameter_test");
  HelperTask task;

  const auto value = task.declare_or_get(node, "demo.value", 42);

  EXPECT_EQ(value, 42);
  EXPECT_TRUE(node->has_parameter("demo.value"));
  EXPECT_EQ(node->get_parameter("demo.value").as_int(), 42);
}

TEST(DiagnosticTaskHelpers, DeclareOrGetReturnsExistingOverride)
{
  auto node = make_node("declare_existing_parameter_test", {rclcpp::Parameter("demo.value", 7)});
  HelperTask task;

  const auto value = task.declare_or_get(node, "demo.value", 42);
  const auto second_value = task.declare_or_get(node, "demo.value", 99);

  EXPECT_EQ(value, 7);
  EXPECT_EQ(second_value, 7);
  EXPECT_EQ(node->get_parameter("demo.value").as_int(), 7);
}

TEST(DiagnosticTaskNames, BuiltInTasksUseDefaultNames)
{
  auto node = make_node("default_task_name_test");

  system_diagnostics::tasks::CpuTask cpu;
  system_diagnostics::tasks::MemoryTask memory;
  system_diagnostics::tasks::StorageTask storage;

  cpu.configure(node, "cpu");
  memory.configure(node, "memory");
  storage.configure(node, "storage");

  EXPECT_EQ(cpu.name(), "system_diagnostics/cpu");
  EXPECT_EQ(memory.name(), "system_diagnostics/memory");
  EXPECT_EQ(storage.name(), "system_diagnostics/storage");
}

TEST(DiagnosticTaskNames, BuiltInTasksUseExplicitNameOverrides)
{
  auto node = make_node("explicit_task_name_test", {
    rclcpp::Parameter("cpu.name", "custom/cpu"),
    rclcpp::Parameter("memory.name", "custom/memory"),
    rclcpp::Parameter("storage.name", "custom/storage"),
  });

  system_diagnostics::tasks::CpuTask cpu;
  system_diagnostics::tasks::MemoryTask memory;
  system_diagnostics::tasks::StorageTask storage;

  cpu.configure(node, "cpu");
  memory.configure(node, "memory");
  storage.configure(node, "storage");

  EXPECT_EQ(cpu.name(), "custom/cpu");
  EXPECT_EQ(memory.name(), "custom/memory");
  EXPECT_EQ(storage.name(), "custom/storage");
}

TEST(CpuTask, ReportsExceededUsageThreshold)
{
  TemporaryDirectory proc("system_diagnostics_cpu_test");
  write_file(proc.path() / "loadavg", "0.00 0.00 0.00 1/1 1\n");
  write_file(proc.path() / "stat", "cpu 100 0 0 100 0 0 0 0 0 0\n");

  auto node = make_node("cpu_threshold_test", {
    rclcpp::Parameter("cpu.proc_path", proc.path().string()),
    rclcpp::Parameter("cpu.error_usage", 90.0),
    rclcpp::Parameter("cpu.error_load_per_core", 1000.0),
    rclcpp::Parameter("cpu.warn_load_per_core", 1000.0),
  });
  system_diagnostics::tasks::CpuTask task;
  task.configure(node, "cpu");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);
  write_file(proc.path() / "stat", "cpu 200 0 0 100 0 0 0 0 0 0\n");
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "CPU usage 100.0% exceeded error_usage 90.0%");
}

TEST(MemoryTask, ReportsExceededMemoryThreshold)
{
  TemporaryDirectory proc("system_diagnostics_memory_test");
  write_file(
    proc.path() / "meminfo",
    "MemTotal: 1000 kB\n"
    "MemAvailable: 40 kB\n"
    "SwapTotal: 1000 kB\n"
    "SwapFree: 1000 kB\n");

  auto node = make_node("memory_threshold_test", {
    rclcpp::Parameter("memory.proc_path", proc.path().string()),
    rclcpp::Parameter("memory.error_usage", 95.0),
  });
  system_diagnostics::tasks::MemoryTask task;
  task.configure(node, "memory");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Memory usage 96.0% exceeded error_usage 95.0%");
}

TEST(StorageTask, ReportsExceededUsageThreshold)
{
  auto node = make_node("storage_threshold_test", {
    rclcpp::Parameter("storage.paths", std::vector<std::string>{"/"}),
    rclcpp::Parameter("storage.error_usage", 0.0),
  });
  system_diagnostics::tasks::StorageTask task;
  task.configure(node, "storage");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("/ used "), std::string::npos);
  EXPECT_NE(status.message.find("exceeded error_usage 0.0%"), std::string::npos);
}

TEST(SystemDiagnosticsNode, PublishesConfiguredStatusNameWithoutNodePrefix)
{
  TemporaryDirectory proc("system_diagnostics_node_name_test");
  write_file(proc.path() / "loadavg", "0.00 0.00 0.00 1/1 1\n");
  write_file(proc.path() / "stat", "cpu 100 0 0 100 0 0 0 0 0 0\n");

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("update_rate", 20.0),
    rclcpp::Parameter("hardware_id", "test_host"),
    rclcpp::Parameter("tasks", std::vector<std::string>{"cpu"}),
    rclcpp::Parameter("cpu.plugin", "system_diagnostics/CpuTask"),
    rclcpp::Parameter("cpu.enabled", true),
    rclcpp::Parameter("cpu.name", "system_diagnostics/cpu"),
    rclcpp::Parameter("cpu.proc_path", proc.path().string()),
  });

  auto diagnostics_node = std::make_shared<system_diagnostics::SystemDiagnosticsNode>(options);
  auto subscriber_node = std::make_shared<rclcpp::Node>("diagnostics_name_subscriber");
  std::optional<diagnostic_msgs::msg::DiagnosticArray> received_message;
  auto subscription = subscriber_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    rclcpp::SystemDefaultsQoS(),
    [&received_message](const diagnostic_msgs::msg::DiagnosticArray & message) {
      received_message = message;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(diagnostics_node->get_node_base_interface());
  executor.add_node(subscriber_node);

  ASSERT_EQ(
    diagnostics_node->on_configure(rclcpp_lifecycle::State()),
    system_diagnostics::SystemDiagnosticsNode::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    diagnostics_node->on_activate(rclcpp_lifecycle::State()),
    system_diagnostics::SystemDiagnosticsNode::CallbackReturn::SUCCESS);

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  auto has_expected_status = [&received_message]() {
      if (!received_message) {
        return false;
      }
      return std::any_of(
        received_message->status.begin(), received_message->status.end(),
        [](const diagnostic_msgs::msg::DiagnosticStatus & status) {
          return status.name == "system_diagnostics/cpu" && status.hardware_id == "test_host";
        });
    };

  while (!has_expected_status() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(50ms);
  }

  ASSERT_TRUE(received_message.has_value());
  ASSERT_TRUE(has_expected_status());
  for (const auto & status : received_message->status) {
    EXPECT_NE(status.name, "system_diagnostics: /system_diagnostics/cpu");
    EXPECT_NE(status.name, "system_diagnostics: system_diagnostics/cpu");
  }

  diagnostics_node->on_deactivate(rclcpp_lifecycle::State());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
