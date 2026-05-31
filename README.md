# system_diagnostics

Plugin-based ROS 2 diagnostic updater for monitoring host system health such as CPU, memory,
storage, thermal state, and network interfaces.

## Features

- Lifecycle component: `system_diagnostics::SystemDiagnosticsNode`
- Standalone executable: `system_diagnostics_node`
- YAML-driven plugin loading through first-level parameter namespaces
- Built-in CPU, memory, and storage diagnostic task plugins
- Custom task plugins through `pluginlib`
- Publishes standard `diagnostic_msgs/msg/DiagnosticArray` messages on `/diagnostics`

## Built-In Task Plugins

The package ships these task plugins:

| Namespace | Plugin |
| --- | --- |
| `cpu` | `system_diagnostics/CpuTask` |
| `memory` | `system_diagnostics/MemoryTask` |
| `storage` | `system_diagnostics/StorageTask` |

The node loads task namespaces that declare a `plugin` parameter. Set
`tasks` to the namespace list to load, and set `<namespace>.enabled: false` to
skip a configured task.

Example:

```yaml
system_diagnostics:
  ros__parameters:
    tasks: ["cpu"]
    cpu:
      plugin: "system_diagnostics/CpuTask"
      enabled: true
```

Custom plugins use the same explicit convention:

```yaml
system_diagnostics:
  ros__parameters:
    tasks: ["gpu"]
    gpu:
      plugin: "my_robot_diagnostics/NvidiaGpuTask"
      warn_temperature: 80.0
```

## Configuration

The default configuration is installed from:

```text
config/system_diagnostics.yaml
```

Top-level parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `update_rate` | `1.0` | Diagnostic update rate in Hz |
| `hardware_id` | `host_pc` | Hardware ID attached to diagnostic statuses |
| `tasks` | `[]` | Ordered list of task namespaces to declare and load |

Task thresholds and source paths are configured under each task namespace. See
`config/system_diagnostics.yaml` for the complete default parameter set.

## Build

From the workspace root:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --base-paths /root/ros2_ws/src --packages-select system_diagnostics --event-handlers console_direct+
```

## Run

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch system_diagnostics system_diagnostics.launch.py
```

Inspect output:

```bash
ros2 topic echo /diagnostics
```

## Component Loading

The package exports the lifecycle component:

```text
system_diagnostics::SystemDiagnosticsNode
```

Use node options that keep undeclared parameter access disabled:

```cpp
rclcpp::NodeOptions()
  .allow_undeclared_parameters(false)
  .use_intra_process_comms(true);
```

`SystemDiagnosticsNode` explicitly declares `tasks`, then declares YAML override
parameters under each listed task namespace.

## Writing Plugins

Implement `system_diagnostics::DiagnosticTask` and export the class with `pluginlib`.
The node calls `configure(node, parameter_namespace)` during lifecycle configure,
registers the task with `diagnostic_updater`, and calls `cleanup()` during lifecycle cleanup.

Plugin load failures are logged as warnings and do not prevent the node from configuring.
