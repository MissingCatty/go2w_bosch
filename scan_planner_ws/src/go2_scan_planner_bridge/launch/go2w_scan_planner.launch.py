"""GO2-W + LIO-SAM integration for SCAN-Planner; dry-run by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    bridge_share = get_package_share_directory('go2_scan_planner_bridge')
    config = os.path.join(bridge_share, 'config', 'go2w.yaml')
    dry_run = _as_bool(LaunchConfiguration('dry_run').perform(context))
    if not dry_run:
        raise RuntimeError(
            'Real cmd_vel output is intentionally disabled in this launch. '
            'Validate /scan_planner/cmd_vel_test first and use a separate safety gate.')

    common_remaps = [
        ('body_pose', '/scan_planner/body_pose'),
        ('sensor_pose', '/scan_planner/sensor_pose'),
        ('cloud', '/scan_planner/local_cloud'),
        ('move_base_simple/goal', '/scan_planner/goal'),
        ('initial_path', '/scan_planner/global_path'),
        ('planning/bspline', '/scan_planner/planning/bspline'),
        ('planning/go2_execution_frozen', '/scan_planner/planning/execution_frozen'),
        ('planning/replan_request', '/scan_planner/planning/replan_request'),
        ('planning/controller_velocity_world', '/scan_planner/planning/controller_velocity_world'),
        ('planning/local_waiting', '/scan_planner/planning/local_waiting'),
        ('planning/local_horizon', '/scan_planner/planning/local_horizon'),
        ('navigation_cancel', '/scan_planner/cancel'),
        ('navigation_completed', '/scan_planner/navigation_completed'),
        ('planning/data_display', '/scan_planner/planning/data_display'),
        ('grid_map/occupancy', '/scan_planner/grid_map/occupancy'),
        ('grid_map/occupancy_inflate', '/scan_planner/grid_map/occupancy_inflate'),
        ('grid_map/sliding_map_bbox', '/scan_planner/grid_map/sliding_map_bbox'),
        ('grid_map/unknown', '/scan_planner/grid_map/unknown'),
        ('grid_map/depth_cloud', '/scan_planner/grid_map/depth_cloud'),
        ('grid_map/sensor_pose_extrinsic', '/scan_planner/grid_map/sensor_pose_extrinsic'),
        ('self_inflation', '/scan_planner/self_inflation'),
    ]
    return [
        Node(
            package='go2_scan_planner_bridge', executable='static_navigation_map',
            name='go2_static_navigation_map', output='screen', parameters=[config]),
        Node(
            package='go2_scan_planner_bridge', executable='lio_pose_adapter',
            name='go2_scan_lio_pose_adapter', output='screen', parameters=[config]),
        Node(
            package='plan_env', executable='near_field_cloud_fuser',
            name='go2_near_field_cloud_fuser', output='screen', parameters=[config]),
        Node(
            package='scan_planner', executable='scan_planner_node',
            name='scan_planner_node', output='screen', parameters=[config],
            remappings=common_remaps),
        Node(
            package='scan_planner', executable='closed_loop_controller',
            name='closed_loop_controller', output='screen', parameters=[config],
            remappings=[
                ('body_pose', '/scan_planner/body_pose'),
                # Emergency braking must use the newest sensor-frame XT16
                # scan. The time-aligned local_cloud intentionally waits for
                # delayed LIO poses and belongs to trajectory planning only.
                ('cloud', '/unitree/slam_lidar/points'),
                ('initial_path', '/scan_planner/global_path'),
                ('planning/bspline', '/scan_planner/planning/bspline'),
                ('planning/local_path', '/scan_planner/local_path'),
                ('planning/go2_execution_frozen', '/scan_planner/planning/execution_frozen'),
                ('planning/replan_request', '/scan_planner/planning/replan_request'),
                ('planning/controller_velocity_world', '/scan_planner/planning/controller_velocity_world'),
                ('planning/emergency_stop', '/scan_planner/emergency_stop'),
                ('navigation_cancel', '/scan_planner/cancel'),
                ('cmd_vel', '/scan_planner/cmd_vel_test'),
            ]),
        Node(
            package='go2_scan_planner_bridge', executable='realtime_recovery',
            name='go2_realtime_recovery', output='screen', parameters=[config]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Must remain true; output is isolated on /scan_planner/cmd_vel_test'),
        OpaqueFunction(function=_setup),
    ])
