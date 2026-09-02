#!/bin/bash
# GO2-W built-in front lidar -> ROS 1 Point-LIO -> existing ROS 2 Web console.
# This mode is mapping-only: SCAN/navigation and the physical chassis stay off.
set -euo pipefail

WS="${GO2_SLAM_WS:-/home/unitree/go2_slam_ws}"
POINTLIO_WS="${GO2_FRONT_POINTLIO_WS:-/home/unitree/front_lidar_pointlio_ws}"
BRIDGE_WS="${GO2_FRONT_BRIDGE_WS:-/home/unitree/front_lidar_bridge_ws}"
WEB_URL="${GO2_WEB_URL:-http://127.0.0.1:8890}"
POINTLIO_BIN="$POINTLIO_WS/devel/lib/point_lio_unilidar/pointlio_mapping"
BRIDGE_BIN="$BRIDGE_WS/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge"
# ROS Foxy setup.bash reads this variable without a nounset-safe default.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

stop_user_unit() {
    local unit="$1"
    systemctl --user --no-block stop "$unit" >/dev/null 2>&1 || true
    systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
    for _ in $(seq 1 50); do
        [ "$(systemctl --user show -p LoadState --value "$unit" 2>/dev/null || true)" = not-found ] && return 0
        sleep 0.1
    done
}

ros2_topic_has_data() {
    local topic="$1"
    (set +o pipefail
     PYTHONUNBUFFERED=1 timeout 3 ros2 topic echo "$topic" --no-arr --no-str 2>/dev/null | grep -m1 -q '^header:')
}

lock_chassis() {
    timeout 3 ros2 topic pub --once /scan_planner/teleop_enable std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /scan_planner/chassis_enable std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /scan_planner/cancel std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true
    sleep 0.5
    local status
    status="$(curl -fsS --max-time 3 "$WEB_URL/api/status")"
    python3 -c '
import json, sys
s = json.load(sys.stdin).get("chassis", {})
out = s.get("output", [999, 999, 999])
if s.get("enabled") or any(abs(float(v)) > 1e-4 for v in out):
    raise SystemExit("底盘未锁定或速度不为零")
' <<<"$status"
}

if [ ! -x "$POINTLIO_BIN" ] || [ ! -x "$BRIDGE_BIN" ]; then
    echo "前置雷达依赖尚未构建：" >&2
    echo "  $POINTLIO_BIN" >&2
    echo "  $BRIDGE_BIN" >&2
    exit 1
fi

set +u
source "$WS/setup_env.sh"
set -u
if ! systemctl --user is-active --quiet go2-slam-host.service || \
   ! curl -fsS --max-time 2 "$WEB_URL/api/status" >/dev/null; then
    echo "==> Web/底盘安全门未运行，先拉起基础服务"
    "$WS/start.sh"
    set +u
    source "$WS/setup_env.sh"
    set -u
fi

echo "==> [1/7] 锁定底盘并停止导航"
lock_chassis
/home/unitree/scan_planner_ws/stop_go2w.sh >/dev/null 2>&1 || true
stop_user_unit go2-scan-planner-dry.service

echo "==> [2/7] 检查前置雷达原始数据"
if ! ros2_topic_has_data /utlidar/cloud || ! ros2_topic_has_data /utlidar/imu; then
    echo "前置雷达 /utlidar/cloud 或 /utlidar/imu 没有数据" >&2
    exit 1
fi

echo "==> [3/7] 停止 XT16/LIO-SAM，避免两套建图争用 CPU 和带宽"
systemctl --user --no-block stop go2-lio-sam.service >/dev/null 2>&1 || true
docker stop -t 5 dddmr_humble >/dev/null 2>&1 || true
stop_user_unit go2-lio-sam.service
stop_user_unit go2-xt16.service
pkill -TERM -x xt16_driver >/dev/null 2>&1 || true

echo "==> [4/7] 按 roscore -> bridge -> Point-LIO 顺序启动"
for unit in go2-front-watchdog.service go2-front-pointlio.service \
            go2-front-ros1-bridge.service go2-front-roscore.service; do
    stop_user_unit "$unit"
done
: >/tmp/go2_front_roscore.log
: >/tmp/go2_front_bridge.log
: >/tmp/go2_front_pointlio.log
: >/tmp/go2_front_watchdog.log
systemd-run --user --unit=go2-front-roscore --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --property=StandardOutput=append:/tmp/go2_front_roscore.log \
    --property=StandardError=append:/tmp/go2_front_roscore.log \
    /bin/bash -lc 'source /opt/ros/noetic/setup.bash; exec roscore' >/dev/null
sleep 2
systemd-run --user --unit=go2-front-ros1-bridge --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --property=StandardOutput=append:/tmp/go2_front_bridge.log \
    --property=StandardError=append:/tmp/go2_front_bridge.log \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash; source '$WS/setup_env.sh'; source '$BRIDGE_WS/install/setup.bash'; export ROS_MASTER_URI=http://127.0.0.1:11311; exec '$BRIDGE_BIN' --bridge-all-1to2-topics" >/dev/null
sleep 2
systemd-run --user --unit=go2-front-pointlio --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --property=StandardOutput=append:/tmp/go2_front_pointlio.log \
    --property=StandardError=append:/tmp/go2_front_pointlio.log \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash; source '$POINTLIO_WS/devel/setup.bash'; export ROS_MASTER_URI=http://127.0.0.1:11311; exec roslaunch point_lio_unilidar mapping_unilidar_l1.launch rviz:=false" >/dev/null

echo "==> [5/7] 将网页实时预览切到前置雷达"
stop_user_unit go2-front-lidar-preview.service
stop_user_unit go2-lidar-preview.service
preview_pids="$(pgrep -f '^/home/unitree/go2_slam_ws/install/go2_imu_bridge/lib/go2_imu_bridge/lidar_preview_bridge( |$)' || true)"
[ -z "$preview_pids" ] || kill -TERM $preview_pids
sleep 1
: >/tmp/go2_front_lidar_preview.log
systemd-run --user --unit=go2-front-lidar-preview --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --property=StandardOutput=append:/tmp/go2_front_lidar_preview.log \
    --property=StandardError=append:/tmp/go2_front_lidar_preview.log \
    /bin/bash -lc "source '$WS/setup_env.sh'; exec ros2 run go2_imu_bridge lidar_preview_bridge --ros-args -r __node:=front_lidar_preview_bridge -r /unitree/slam_lidar/points:=/utlidar/cloud" >/dev/null

echo "==> [6/7] 等待地图输出并启动断流保护"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
ready=false
for _ in $(seq 1 15); do
    if timeout 4 rostopic echo -n 1 /lio_sam/mapping/odometry >/dev/null 2>&1 && \
       timeout 4 rostopic echo -n 1 /lio_sam/mapping/map_global >/dev/null 2>&1; then
        ready=true
        break
    fi
    sleep 0.25
done
if ! $ready; then
    echo "Point-LIO 未输出地图，请检查 /tmp/go2_front_pointlio.log" >&2
    exit 1
fi
systemd-run --user --unit=go2-front-watchdog --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --property=StandardOutput=append:/tmp/go2_front_watchdog.log \
    --property=StandardError=append:/tmp/go2_front_watchdog.log \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; exec python3 '$WS/front_lidar_watchdog.py'" >/dev/null

echo "==> [7/7] 切到建图页并做安全验收"
set +u
source "$WS/setup_env.sh"
set -u
curl -fsS --max-time 5 -H 'Content-Type: application/json' \
    --data '{"mode":"mapping"}' "$WEB_URL/api/mode" >/dev/null || true
lock_chassis
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "============================================================"
echo "  前置雷达 Point-LIO 已就绪（建图默认仍为关闭）"
echo "  打开 http://${IP}:8890 ，点击“开始建图”后测试"
echo "  清除按钮会只重启前置雷达地图；不会删除已保存地图"
echo "  日志: /tmp/go2_front_{pointlio,bridge,watchdog}.log"
echo "============================================================"
