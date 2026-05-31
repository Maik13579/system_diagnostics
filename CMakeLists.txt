# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(system_diagnostics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_library(system_diagnostics_core SHARED
  src/system_diagnostics_node.cpp
)
target_include_directories(system_diagnostics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(system_diagnostics_core
  diagnostic_msgs
  diagnostic_updater
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)
rclcpp_components_register_nodes(system_diagnostics_core
  "system_diagnostics::SystemDiagnosticsNode")

add_library(system_diagnostics_tasks SHARED
  src/tasks/battery_task.cpp
  src/tasks/cpu_task.cpp
  src/tasks/memory_task.cpp
  src/tasks/network_task.cpp
  src/tasks/storage_task.cpp
  src/tasks/thermal_task.cpp
  src/tasks/time_sync_task.cpp
)
target_include_directories(system_diagnostics_tasks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(system_diagnostics_tasks
  diagnostic_msgs
  diagnostic_updater
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(system_diagnostics_node src/main.cpp)
target_link_libraries(system_diagnostics_node system_diagnostics_core)
ament_target_dependencies(system_diagnostics_node rclcpp)

pluginlib_export_plugin_description_file(system_diagnostics plugins.xml)

install(
  TARGETS
    system_diagnostics_core
    system_diagnostics_tasks
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS system_diagnostics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(system_diagnostics_core system_diagnostics_tasks)
ament_export_dependencies(
  diagnostic_msgs
  diagnostic_updater
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  ament_add_gtest(test_diagnostic_tasks
    test/test_diagnostic_tasks.cpp
  )
  target_link_libraries(test_diagnostic_tasks system_diagnostics_core system_diagnostics_tasks)
  ament_target_dependencies(test_diagnostic_tasks
    diagnostic_msgs
    diagnostic_updater
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_package()
