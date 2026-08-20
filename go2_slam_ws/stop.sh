#!/bin/bash
# 停止 GO2-W LIO-SAM 全链路；已有 maps/ 文件不会删除。
set +e

stop_exact_process() {
    local name="$1" pids
    pkill -TERM -x "$name" 2>/dev/null
    sleep 1
    pids="$(pgrep -x "$name" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        docker run --rm --pid=host --privileged dddmr_humble:nav \
            bash -lc "kill -TERM $pids" >/dev/null 2>&1
        sleep 2
    fi
}

echo "==> 停止 Humble LIO-SAM"
systemctl --user --no-block stop go2-lio-sam.service 2>/dev/null
docker stop -t 10 dddmr_humble >/dev/null 2>&1
systemctl --user reset-failed go2-lio-sam.service 2>/dev/null

echo "==> 停止 host ROS2 节点"
systemctl --user stop go2-slam-host.service 2>/dev/null
pkill -f '/opt/ros/foxy/bin/ros2 launch go2_slam_bringup slam.launch.py' 2>/dev/null
pkill -x web_server 2>/dev/null
pkill -f '/go2_slam_core/imu_attitude_bridge' 2>/dev/null
pkill -f '/go2_imu_bridge/imu_attitude_bridge' 2>/dev/null
pkill -f '/go2_imu_bridge/lidar_preview_bridge' 2>/dev/null

# 清理旧版本可能残留的另一套建图链。
stop_exact_process slam_manager
stop_exact_process fallback_slam
stop_exact_process unitree_slam

echo "==> 停止 XT16 驱动"
systemctl --user stop go2-xt16.service 2>/dev/null
stop_exact_process xt16_driver

RUN_TAG="${UID:-$(id -u)}"
rm -f "/tmp/go2_slam_launch_${RUN_TAG}.pid" \
      "/tmp/go2_dddmr_lio_${RUN_TAG}.pid" \
      "/tmp/go2_xt16_driver_${RUN_TAG}.pid"
echo "==> 已停止（已保存地图未改动）"
exit 0
