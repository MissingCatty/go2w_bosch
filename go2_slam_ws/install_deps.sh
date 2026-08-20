#!/bin/bash
# ============================================================
# 可选依赖安装（增强备用建图方案，非必须）
#   slam_toolbox + pointcloud_to_laserscan 是 ROS2 通用 2D SLAM，
#   安装后 web 界面可自动使用它们输出的 /map 作为地图源。
#   需要输入 sudo 密码。
# ============================================================
echo "安装可选依赖: ros-foxy-slam-toolbox ros-foxy-pointcloud-to-laserscan"
sudo apt update
sudo apt install -y ros-foxy-slam-toolbox ros-foxy-pointcloud-to-laserscan
echo "完成。现在可运行: ./start.sh"
