#!/bin/bash
# Clear only the unfinished front-lidar Point-LIO map. Saved NPZ maps remain.
set -euo pipefail

UNIT="go2-front-pointlio.service"
if ! systemctl --user is-active --quiet "$UNIT"; then
    echo "前置雷达 Point-LIO 未运行" >&2
    exit 1
fi

systemctl --user restart "$UNIT"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
for _ in $(seq 1 80); do
    if timeout 4 rostopic echo -n 1 /lio_sam/mapping/odometry >/dev/null 2>&1; then
        echo "前置雷达当前建图已清除，Point-LIO 已重新初始化"
        exit 0
    fi
    sleep 0.25
done

echo "Point-LIO 重启后未输出里程计，请检查 /tmp/go2_front_pointlio.log" >&2
exit 1
