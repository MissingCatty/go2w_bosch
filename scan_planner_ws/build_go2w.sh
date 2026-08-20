#!/bin/bash
# Build the ROS 2 SCAN-Planner port and the GO2-W adapter in the Humble image.
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
IMAGE="${SCAN_PLANNER_IMAGE:-dddmr_humble:nav}"

docker run --rm --network=host \
  --env=PYTHONDONTWRITEBYTECODE=1 \
  --volume="$WS:/root/scan_planner_ws" \
  "$IMAGE" bash -lc '
    set -eo pipefail
    apt-get update -qq
    apt-get install -y -qq libglm-dev
    source /opt/ros/humble/setup.bash
    cd /root/scan_planner_ws
    export MAKEFLAGS=-j2
    colcon build --symlink-install \
      --packages-select \
        scan_planner_msgs plan_env path_searching bspline_opt traj_utils \
        go2_description map_generator mockamap pose_utils odom_visualization \
        local_sensing_node scan_planner go2_scan_planner_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DUSE_GPU=OFF
  '
