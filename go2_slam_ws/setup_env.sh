#!/bin/bash
# go2_slam_ws 统一 ROS2 环境（修复本机 ROS1 noetic 与 ROS2 foxy 混装导致的 python 包冲突）
# 用法: source ~/go2_slam_ws/setup_env.sh
unset PYTHONPATH ROS_PACKAGE_PATH ROS_MASTER_URI ROS_DISTRO
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/foxy/setup.bash 2>/dev/null
source /home/unitree/cyclonedds_ws/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
# ROS_DOMAIN_ID 统一为 0（默认值，显式固定避免环境差异）
export ROS_DOMAIN_ID=0
# 强制 foxy 的 python 路径在最前，避免 noetic 的 std_msgs 等遮蔽 ROS2 消息
export PYTHONPATH="/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}"
# 本工作空间的 install（如有）
_GO2_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$_GO2_WS/install/setup.bash" ] && source "$_GO2_WS/install/setup.bash" 2>/dev/null
