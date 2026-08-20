#!/bin/bash
# Send a planning-only goal; /scan_planner/cmd_vel_test remains disconnected from the robot.
set -euo pipefail
DISTANCE="${1:-1.0}"
YAW_OFFSET_DEG="${2:-0.0}"
docker exec go2_scan_planner_dry bash -lc \
  "source /opt/ros/humble/setup.bash; source /root/scan_planner_ws/install/setup.bash; ros2 run go2_scan_planner_bridge forward_test_goal --ros-args -p distance:=$DISTANCE -p yaw_offset_deg:=$YAW_OFFSET_DEG"
