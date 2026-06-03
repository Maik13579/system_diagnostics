// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0

#include "system_diagnostics/diagnostic_task.hpp"
#include "system_diagnostics/system_diagnostics_node.hpp"
#include "system_diagnostics/tasks/cpu_task.hpp"
#include "system_diagnostics/tasks/battery_task.hpp"
#include "system_diagnostics/tasks/docker_memory_task.hpp"
#include "system_diagnostics/tasks/memory_task.hpp"
#include "system_diagnostics/tasks/network_task.hpp"
#include "system_diagnostics/tasks/storage_task.hpp"
#include "system_diagnostics/tasks/thermal_task.hpp"
#include "system_diagnostics/tasks/time_sync_task.hpp"

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

void make_network_interface(
  const std::filesystem::path & root,
  const std::string & name,
  const std::string & carrier,
  const std::string & operstate,
  std::uint64_t rx_errors,
  std::uint64_t tx_errors,
  std::uint64_t rx_dropped,
  std::uint64_t tx_dropped)
{
  const auto interface_path = root / name;
  std::filesystem::create_directories(interface_path / "statistics");
  write_file(interface_path / "carrier", carrier + "\n");
  write_file(interface_path / "operstate", operstate + "\n");
  write_file(interface_path / "statistics" / "rx_errors", std::to_string(rx_errors) + "\n");
  write_file(interface_path / "statistics" / "tx_errors", std::to_string(tx_errors) + "\n");
  write_file(interface_path / "statistics" / "rx_dropped", std::to_string(rx_dropped) + "\n");
  write_file(interface_path / "statistics" / "tx_dropped", std::to_string(tx_dropped) + "\n");
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
  system_diagnostics::tasks::DockerMemoryTask docker_memory;
  system_diagnostics::tasks::StorageTask storage;
  system_diagnostics::tasks::ThermalTask thermal;
  system_diagnostics::tasks::NetworkTask network;
  system_diagnostics::tasks::BatteryTask battery;
  system_diagnostics::tasks::TimeSyncTask time_sync;

  cpu.configure(node, "cpu");
  memory.configure(node, "memory");
  docker_memory.configure(node, "docker_memory");
  storage.configure(node, "storage");
  thermal.configure(node, "thermal");
  network.configure(node, "network");
  battery.configure(node, "battery");
  time_sync.configure(node, "time_sync");

  EXPECT_EQ(cpu.name(), "system_diagnostics/cpu");
  EXPECT_EQ(memory.name(), "system_diagnostics/memory");
  EXPECT_EQ(docker_memory.name(), "system_diagnostics/docker_memory");
  EXPECT_EQ(storage.name(), "system_diagnostics/storage");
  EXPECT_EQ(thermal.name(), "system_diagnostics/thermal");
  EXPECT_EQ(network.name(), "system_diagnostics/network");
  EXPECT_EQ(battery.name(), "system_diagnostics/battery");
  EXPECT_EQ(time_sync.name(), "system_diagnostics/time_sync");
}

TEST(DiagnosticTaskNames, BuiltInTasksUseExplicitNameOverrides)
{
  auto node = make_node("explicit_task_name_test", {
    rclcpp::Parameter("cpu.name", "custom/cpu"),
    rclcpp::Parameter("memory.name", "custom/memory"),
    rclcpp::Parameter("docker_memory.name", "custom/docker_memory"),
    rclcpp::Parameter("storage.name", "custom/storage"),
    rclcpp::Parameter("thermal.name", "custom/thermal"),
    rclcpp::Parameter("network.name", "custom/network"),
    rclcpp::Parameter("battery.name", "custom/battery"),
    rclcpp::Parameter("time_sync.name", "custom/time_sync"),
  });

  system_diagnostics::tasks::CpuTask cpu;
  system_diagnostics::tasks::MemoryTask memory;
  system_diagnostics::tasks::DockerMemoryTask docker_memory;
  system_diagnostics::tasks::StorageTask storage;
  system_diagnostics::tasks::ThermalTask thermal;
  system_diagnostics::tasks::NetworkTask network;
  system_diagnostics::tasks::BatteryTask battery;
  system_diagnostics::tasks::TimeSyncTask time_sync;

  cpu.configure(node, "cpu");
  memory.configure(node, "memory");
  docker_memory.configure(node, "docker_memory");
  storage.configure(node, "storage");
  thermal.configure(node, "thermal");
  network.configure(node, "network");
  battery.configure(node, "battery");
  time_sync.configure(node, "time_sync");

  EXPECT_EQ(cpu.name(), "custom/cpu");
  EXPECT_EQ(memory.name(), "custom/memory");
  EXPECT_EQ(docker_memory.name(), "custom/docker_memory");
  EXPECT_EQ(storage.name(), "custom/storage");
  EXPECT_EQ(thermal.name(), "custom/thermal");
  EXPECT_EQ(network.name(), "custom/network");
  EXPECT_EQ(battery.name(), "custom/battery");
  EXPECT_EQ(time_sync.name(), "custom/time_sync");
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

TEST(DockerMemoryTask, ReportsWorstContainerOverThreshold)
{
  auto node = make_node("docker_memory_threshold_test", {
    rclcpp::Parameter("docker_memory.error_usage", 90.0),
  });
  system_diagnostics::tasks::DockerMemoryTask task;
  task.configure(node, "docker_memory");
  task.set_sampler([]() {
      system_diagnostics::tasks::DockerMemoryTask::Sample sample;
      sample.success = true;
      sample.containers = {
        {"abc123", "nav", 950, 1000},
        {"def456", "velodyne", 400, 1000},
      };
      return sample;
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "nav memory usage 95.0% exceeded error_usage 90.0%");
  const auto nav_value = std::find_if(
    status.values.begin(), status.values.end(),
    [](const diagnostic_msgs::msg::KeyValue & value) {
      return value.key == "nav";
    });
  ASSERT_NE(nav_value, status.values.end());
  EXPECT_EQ(nav_value->value, "950 B");
}

TEST(DockerMemoryTask, ReportsSocketError)
{
  auto node = make_node("docker_memory_socket_error_test");
  system_diagnostics::tasks::DockerMemoryTask task;
  task.configure(node, "docker_memory");
  task.set_sampler([]() {
      system_diagnostics::tasks::DockerMemoryTask::Sample sample;
      sample.success = false;
      sample.error = "connect failed for /var/run/docker.sock: No such file or directory";
      return sample;
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "connect failed for /var/run/docker.sock: No such file or directory");
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

TEST(ThermalTask, ReportsWorstExceededZoneThreshold)
{
  TemporaryDirectory sysfs("system_diagnostics_thermal_test");
  std::filesystem::create_directories(sysfs.path() / "thermal_zone0");
  std::filesystem::create_directories(sysfs.path() / "thermal_zone1");
  write_file(sysfs.path() / "thermal_zone0" / "type", "cpu_thermal\n");
  write_file(sysfs.path() / "thermal_zone0" / "temp", "76000\n");
  write_file(sysfs.path() / "thermal_zone1" / "type", "x86_pkg_temp\n");
  write_file(sysfs.path() / "thermal_zone1" / "temp", "91200\n");

  auto node = make_node("thermal_threshold_test", {
    rclcpp::Parameter("thermal.sysfs_path", sysfs.path().string()),
    rclcpp::Parameter("thermal.warn_temperature_c", 75.0),
    rclcpp::Parameter("thermal.error_temperature_c", 90.0),
  });
  system_diagnostics::tasks::ThermalTask task;
  task.configure(node, "thermal");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "x86_pkg_temp 91.2 C exceeded error_temperature_c 90.0 C");
}

TEST(ThermalTask, MissingEnabledSourceIsError)
{
  TemporaryDirectory sysfs("system_diagnostics_thermal_missing_test");
  auto node = make_node("thermal_missing_test", {
    rclcpp::Parameter("thermal.sysfs_path", sysfs.path().string()),
  });
  system_diagnostics::tasks::ThermalTask task;
  task.configure(node, "thermal");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "No thermal zones found in " + sysfs.path().string());
}

TEST(NetworkTask, ReportsCarrierDown)
{
  TemporaryDirectory sysfs("system_diagnostics_network_carrier_test");
  make_network_interface(sysfs.path(), "eth_test", "0", "down", 0, 0, 0, 0);

  auto node = make_node("network_carrier_test", {
    rclcpp::Parameter("network.sysfs_path", sysfs.path().string()),
    rclcpp::Parameter("network.interfaces", std::vector<std::string>{"eth_test"}),
    rclcpp::Parameter("network.require_carrier", true),
  });
  system_diagnostics::tasks::NetworkTask task;
  task.configure(node, "network");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "eth_test carrier is down");
}

TEST(NetworkTask, ReportsCounterDeltaThreshold)
{
  TemporaryDirectory sysfs("system_diagnostics_network_delta_test");
  make_network_interface(sysfs.path(), "eth_test", "1", "up", 0, 0, 0, 0);

  auto node = make_node("network_delta_test", {
    rclcpp::Parameter("network.sysfs_path", sysfs.path().string()),
    rclcpp::Parameter("network.interfaces", std::vector<std::string>{"eth_test"}),
    rclcpp::Parameter("network.error_rx_errors_delta", 10),
  });
  system_diagnostics::tasks::NetworkTask task;
  task.configure(node, "network");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);
  make_network_interface(sysfs.path(), "eth_test", "1", "up", 12, 0, 0, 0);
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "eth_test rx_errors delta 12 exceeded error_rx_errors_delta 10");
}

TEST(BatteryTask, ReportsLowCapacityThreshold)
{
  TemporaryDirectory sysfs("system_diagnostics_battery_threshold_test");
  std::filesystem::create_directories(sysfs.path() / "BAT_TEST");
  write_file(sysfs.path() / "BAT_TEST" / "present", "1\n");
  write_file(sysfs.path() / "BAT_TEST" / "capacity", "12\n");
  write_file(sysfs.path() / "BAT_TEST" / "status", "Discharging\n");

  auto node = make_node("battery_threshold_test", {
    rclcpp::Parameter("battery.power_supply_path", sysfs.path().string()),
    rclcpp::Parameter("battery.supplies", std::vector<std::string>{"BAT_TEST"}),
    rclcpp::Parameter("battery.error_capacity", 15.0),
  });
  system_diagnostics::tasks::BatteryTask task;
  task.configure(node, "battery");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "BAT_TEST capacity 12.0% below error_capacity 15.0%");
}

TEST(BatteryTask, ReportsMissingSupply)
{
  TemporaryDirectory sysfs("system_diagnostics_battery_missing_test");
  auto node = make_node("battery_missing_test", {
    rclcpp::Parameter("battery.power_supply_path", sysfs.path().string()),
    rclcpp::Parameter("battery.supplies", std::vector<std::string>{"BAT_TEST"}),
  });
  system_diagnostics::tasks::BatteryTask task;
  task.configure(node, "battery");

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "Battery supply does not exist: " + (sysfs.path() / "BAT_TEST").string());
}

TEST(TimeSyncTask, ReportsUnsynchronizedState)
{
  auto node = make_node("time_sync_unsynchronized_test", {
    rclcpp::Parameter("time_sync.require_synchronized", true),
  });
  system_diagnostics::tasks::TimeSyncTask task;
  task.configure(node, "time_sync");
  task.set_sampler([]() {
      system_diagnostics::tasks::TimeSyncTask::Sample sample;
      sample.success = true;
      sample.synchronized = false;
      sample.status_bits = 64;
      return sample;
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Time synchronization is unsynchronized");
}

TEST(TimeSyncTask, ReportsExceededOffsetThreshold)
{
  auto node = make_node("time_sync_threshold_test", {
    rclcpp::Parameter("time_sync.error_offset_ms", 100.0),
  });
  system_diagnostics::tasks::TimeSyncTask task;
  task.configure(node, "time_sync");
  task.set_sampler([]() {
      system_diagnostics::tasks::TimeSyncTask::Sample sample;
      sample.success = true;
      sample.synchronized = true;
      sample.offset_ms = -125.5;
      return sample;
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Time offset 125.500 ms exceeded error_offset_ms 100.000 ms");
}

TEST(TimeSyncTask, ReportsSynchronizedState)
{
  auto node = make_node("time_sync_ok_test");
  system_diagnostics::tasks::TimeSyncTask task;
  task.configure(node, "time_sync");
  task.set_sampler([]() {
      system_diagnostics::tasks::TimeSyncTask::Sample sample;
      sample.success = true;
      sample.synchronized = true;
      sample.offset_ms = 1.0;
      sample.max_error_ms = 2.0;
      sample.estimated_error_ms = 1.0;
      return sample;
    });

  diagnostic_updater::DiagnosticStatusWrapper status;
  task.update(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Time synchronization OK");
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
