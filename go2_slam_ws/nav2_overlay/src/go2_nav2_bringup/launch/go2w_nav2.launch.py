#!/usr/bin/env python3
"""Launch GO2-W Nav2 in a command-isolated shadow graph."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('go2_nav2_bringup')
    default_params = os.path.join(package_dir, 'config', 'nav2_go2w.yaml')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        # Global nodes come last. After a reboot the validated nav_map->odom
        # transform intentionally does not exist yet; local safety nodes can
        # still become active while planner activation waits for alignment.
        'planner_server',
        'bt_navigator',
    ]
    common = {
        'output': 'screen',
        'parameters': [params_file],
        'arguments': ['--ros-args', '--log-level', log_level],
    }
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='GO2-W Nav2 parameter file'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', remappings=tf_remaps, **common),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server',
            remappings=tf_remaps + [
                ('cmd_vel', '/go2/nav2/cmd_vel_raw'),
            ], **common),
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', remappings=tf_remaps, **common),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', remappings=tf_remaps, **common),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server',
            remappings=tf_remaps + [
                ('cmd_vel', '/go2/nav2/cmd_vel_raw'),
            ], **common),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', remappings=tf_remaps, **common),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother',
            remappings=tf_remaps + [
                ('cmd_vel', '/go2/nav2/cmd_vel_raw'),
                ('cmd_vel_smoothed', '/go2/nav2/cmd_vel_smoothed'),
            ], **common),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', remappings=tf_remaps, **common),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_go2_nav2', output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
                'node_names': lifecycle_nodes,
            }],
            arguments=['--ros-args', '--log-level', log_level]),
        Node(
            package='go2_nav2_bringup', executable='map_odom_bridge',
            name='go2_nav2_map_odom_bridge', output='screen',
            arguments=['--ros-args', '--log-level', log_level]),
        Node(
            package='go2_nav2_bringup', executable='odom_qos_bridge',
            name='go2_nav2_odom_qos_bridge', output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level]),
        Node(
            package='go2_nav2_bringup', executable='nav_goal_bridge',
            name='go2_nav2_goal_bridge', output='screen',
            arguments=['--ros-args', '--log-level', log_level]),
        Node(
            package='go2_nav2_bringup', executable='shadow_monitor',
            name='go2_nav2_shadow_monitor', output='screen',
            arguments=['--ros-args', '--log-level', log_level]),
    ])
