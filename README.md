# system_diagnostics

Plugin-based ROS 2 diagnostic updater for monitoring host system health such as CPU, memory,
storage, thermal state, and network interfaces.

## Features

- Lifecycle component: `system_diagnostics::SystemDiagnosticsNode`
- Standalone executable: `system_diagnostics_node`
- YAML-driven plugin loading through first-level parameter namespaces
- Built-in CPU, memory, Docker memory, storage, thermal, network, battery, and time-sync diagnostic task plugins
- Custom task plugins through `pluginlib`
- Publishes standard `diagnostic_msgs/msg/DiagnosticArray` messages on `/diagnostics`

This package builds on the standard ROS 2 diagnostics stack. See the upstream
[`ros/diagnostics`](https://github.com/ros/diagnostics/tree/ros2) repository for
the `diagnostic_updater` and related diagnostic tools.

## Built-In Task Plugins

The package ships these task plugins:

| Namespace | Plugin | Default diagnostic name | Parameters |
| --------- | ------ | ----------------------- | ---------- |
| `cpu` | `system_diagnostics/CpuTask` | `system_diagnostics/cpu` | `enabled`, `name`, `warn_usage`, `error_usage`, `warn_load_per_core`, `error_load_per_core`, `proc_path` |
| `memory` | `system_diagnostics/MemoryTask` | `system_diagnostics/memory` | `enabled`, `name`, `warn_usage`, `error_usage`, `warn_swap_usage`, `error_swap_usage`, `proc_path` |
| `docker_memory` | `system_diagnostics/DockerMemoryTask` | `system_diagnostics/docker_memory` | `enabled`, `name`, `socket_path`, `warn_usage`, `error_usage`, `warn_usage_bytes`, `error_usage_bytes` |
| `storage` | `system_diagnostics/StorageTask` | `system_diagnostics/storage` | `enabled`, `name`, `paths`, `warn_usage`, `error_usage`, `check_writable` |
| `thermal` | `system_diagnostics/ThermalTask` | `system_diagnostics/thermal` | `enabled`, `name`, `sysfs_path`, `zones`, `warn_temperature_c`, `error_temperature_c` |
| `network` | `system_diagnostics/NetworkTask` | `system_diagnostics/network` | `enabled`, `name`, `sysfs_path`, `interfaces`, `require_carrier`, RX/TX error and drop delta thresholds |
| `time_sync` | `system_diagnostics/TimeSyncTask` | `system_diagnostics/time_sync` | `enabled`, `name`, `require_synchronized`, offset and max-error thresholds in milliseconds |
| `battery` | `system_diagnostics/BatteryTask` | `system_diagnostics/battery` | `enabled`, `name`, `power_supply_path`, `supplies`, `warn_capacity`, `error_capacity`, `require_present` |

The default YAML enables `thermal`, `network`, and `time_sync` with strict
source checks. `docker_memory` and `battery` are configured but disabled by
default to avoid noisy desktop or container deployments. Enable `docker_memory`
after mounting `/var/run/docker.sock` into the diagnostics container. It checks
all running containers and emits a direct diagnostic key-value for each one, for example
`nav: 178.8 MiB`, plus detailed `container.<name>.*` values. Omit `thermal.zones` to discover all
`thermal_zone*` entries under `thermal.sysfs_path`; set it only when selecting
specific zones.

The node loads task namespaces that declare a `plugin` parameter. Set
`tasks` to the namespace list to load, and set `<namespace>.enabled: false` to
skip a configured task. Set `<namespace>.name` to control the diagnostic status
name. Built-in defaults use `system_diagnostics/<namespace>`.

Example:

```yaml
system_diagnostics:
  ros__parameters:
    tasks: ["cpu"]
    cpu:
      plugin: "system_diagnostics/CpuTask"
      enabled: true
      name: "system_diagnostics/cpu"
```

Custom plugins use the same explicit convention:

```yaml
system_diagnostics:
  ros__parameters:
    tasks: ["gpu"]
    gpu:
      plugin: "my_robot_diagnostics/NvidiaGpuTask"
      enabled: true
      name: "system_diagnostics/gpu"
      warn_temperature: 80.0
```

## Configuration

The default configuration is installed from:

```text
config/system_diagnostics.yaml
```

Top-level parameters:

| Parameter     | Default   | Description                                         |
| ------------- | --------- | --------------------------------------------------- |
| `update_rate` | `1.0`     | Diagnostic update rate in Hz                        |
| `num_threads` | `1`       | Number of worker threads used to update tasks       |
| `hardware_id` | `host_pc` | Hardware ID attached to diagnostic statuses         |
| `tasks`       | Built-ins | Ordered list of task namespaces to declare and load |

Task thresholds, source paths, and diagnostic names are configured under each
task namespace. The configured `name` is the human-facing diagnostic status name
emitted on `/diagnostics`; no node-name prefix is added. See
`config/system_diagnostics.yaml` for the complete default parameter set.

Each published `DiagnosticArray` also includes `system_diagnostics/update_timing`
with one key/value per task namespace, an `all` total duration in milliseconds,
and an ERROR summary when the cycle takes longer than the configured period.

## Writing Plugins

Implement `system_diagnostics::DiagnosticTask` and export the class with `pluginlib`.
The node calls `configure(node, parameter_namespace)` during lifecycle configure,
registers the task with `diagnostic_updater`, and calls `cleanup()` during
lifecycle cleanup.

Recommended task structure:

1. Inherit from `system_diagnostics::DiagnosticTask`.
2. In `configure()`, call `configure_name(node, parameter_namespace, default_name)`
   before reading task-specific parameters.
3. Use `declare_or_get(node, parameter_namespace + ".parameter", default_value)`
   for task parameters so YAML overrides and defaults share the same path.
4. Implement `cleanup()` to release task-owned resources and transient state.
5. Implement `update()` to fill a `diagnostic_updater::DiagnosticStatusWrapper`.
6. Export the task with `PLUGINLIB_EXPORT_CLASS(MyTask, system_diagnostics::DiagnosticTask)`.
7. Add the class to plugin XML and add YAML entries for `plugin`, `enabled`,
   `name`, and task-specific parameters.

Example plugin XML entry:

```xml
<class
  name="my_robot_diagnostics/NvidiaGpuTask"
  type="my_robot_diagnostics::NvidiaGpuTask"
  base_class_type="system_diagnostics::DiagnosticTask">
  <description>Reports NVIDIA GPU diagnostics.</description>
</class>
```

Example YAML entry:

```yaml
system_diagnostics:
  ros__parameters:
    tasks: ["gpu"]
    gpu:
      plugin: "my_robot_diagnostics/NvidiaGpuTask"
      enabled: true
      name: "system_diagnostics/gpu"
      warn_temperature: 80.0
```

Plugin load failures are logged as warnings and do not prevent the node from configuring.
