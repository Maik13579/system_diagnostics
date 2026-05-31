# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
            FindPackageShare('system_diagnostics'),
            'config',
            'system_diagnostics.yaml',
        ]),
        description='Path to the system diagnostics parameter file',
    )

    return LaunchDescription([
        config_file,
        Node(
            package='system_diagnostics',
            executable='system_diagnostics_node',
            name='system_diagnostics',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
