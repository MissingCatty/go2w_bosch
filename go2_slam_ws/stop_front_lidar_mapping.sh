#!/bin/bash
# Stop the front-lidar experiment. Saved NPZ maps are never removed.
set +e

WS="${GO2_SLAM_WS:-/home/unitree/go2_slam_ws}"
for unit in go2-front-watchdog.service go2-front-lidar-preview.service \
            go2-front-pointlio.service go2-front-ros1-bridge.service \
            go2-front-roscore.service; do
    systemctl --user --no-block stop "$unit" >/dev/null 2>&1
    systemctl --user reset-failed "$unit" >/dev/null 2>&1
done
sleep 2

# Restore the normal XT16 Web preview process if the host Web stack is alive.
if systemctl --user is-active --quiet go2-slam-host.service && \
   ! pgrep -f '^/home/unitree/go2_slam_ws/install/go2_imu_bridge/lib/go2_imu_bridge/lidar_preview_bridge( |$)' >/dev/null; then
    source "$WS/setup_env.sh"
    systemctl --user stop go2-lidar-preview.service >/dev/null 2>&1
    systemd-run --user --unit=go2-lidar-preview --collect \
        --property=Restart=on-failure --property=RestartSec=2 \
        --property=StandardOutput=append:/tmp/go2_lidar_preview.log \
        --property=StandardError=append:/tmp/go2_lidar_preview.log \
        /bin/bash -lc "source '$WS/setup_env.sh'; exec ros2 run go2_imu_bridge lidar_preview_bridge" >/dev/null
fi

echo "前置雷达建图已停止；已保存地图未改动。"
echo "要恢复 XT16/LIO-SAM，请运行: $WS/start.sh"
