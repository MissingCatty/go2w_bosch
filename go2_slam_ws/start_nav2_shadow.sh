#!/bin/bash
# Start Nav2 with SCAN as its local-trajectory generator. The gate stays locked.
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
SCAN_WS="/home/unitree/scan_planner_ws"
IMAGE="${GO2_NAV2_IMAGE:-go2_nav2_humble:local}"
CONTAINER="go2_nav2_shadow"
UNIT="go2-nav2-shadow.service"
LOG="/tmp/go2_nav2_shadow_$(id -u).log"
WEB_URL="${GO2_WEB_URL:-http://127.0.0.1:8890}"
CYCLONE_CONFIG="/home/unitree/cyclonedds_ws/cyclonedds.xml"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1 || \
   [ ! -f "$WS/nav2_overlay/install/setup.bash" ]; then
    "$WS/build_nav2.sh"
fi
if [ ! -f "$SCAN_WS/install/setup.bash" ]; then
    echo "错误: SCAN-Planner 消息接口尚未构建: $SCAN_WS/install/setup.bash" >&2
    exit 1
fi
if [ ! -r "$CYCLONE_CONFIG" ]; then
    echo "错误: CycloneDDS 配置不存在: $CYCLONE_CONFIG" >&2
    exit 1
fi

status="$(curl -fsS --max-time 3 "$WEB_URL/api/status" 2>/dev/null || true)"
if ! python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
except Exception:
    raise SystemExit(1)
chassis = data.get("chassis", {})
health = data.get("health", {})
raise SystemExit(0 if (not chassis.get("enabled") and
                       health.get("pose_online") and
                       health.get("pose_valid")) else 1)
' <<<"$status"; then
    echo "错误: 启动 Nav2 前必须保证 LIO 位姿在线且真实底盘已锁定" >&2
    exit 1
fi

for unit in go2-lio-sam.service go2-scan-planner-dry.service; do
    if ! systemctl --user is-active --quiet "$unit"; then
        echo "错误: 基础服务未运行: $unit" >&2
        exit 1
    fi
done

systemctl --user stop "$UNIT" >/dev/null 2>&1 || true
docker stop -t 5 "$CONTAINER" >/dev/null 2>&1 || true
systemctl --user reset-failed "$UNIT" >/dev/null 2>&1 || true
for _ in $(seq 1 30); do
    load_state="$(systemctl --user show "$UNIT" -p LoadState --value 2>/dev/null || true)"
    [ -z "$load_state" ] || [ "$load_state" = "not-found" ] && break
    sleep 0.1
done
: >"$LOG"

systemd-run --user --unit="${UNIT%.service}" --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --property=TimeoutStopSec=8 \
    --property="StandardOutput=append:$LOG" \
    --property="StandardError=append:$LOG" \
    docker run --rm --name "$CONTAINER" --network=host --ipc=host \
      --env=PYTHONDONTWRITEBYTECODE=1 \
      --env=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      --env=CYCLONEDDS_URI=file:///root/cyclonedds.xml \
      --volume="/tmp:/tmp" \
      --volume="$CYCLONE_CONFIG:/root/cyclonedds.xml:ro" \
      --volume="$WS:/root/go2_slam_ws" \
      --volume="$SCAN_WS:/root/scan_planner_ws:ro" \
      "$IMAGE" bash -lc '
        set -e
        source /opt/ros/humble/setup.bash
        source /root/scan_planner_ws/install/setup.bash
        source /root/go2_slam_ws/nav2_overlay/install/setup.bash
        exec ros2 launch go2_nav2_bringup go2w_nav2.launch.py
      ' >/dev/null

alignment_json="$(curl -fsS --max-time 3 "$WEB_URL/api/navigation/alignment" \
    2>/dev/null || true)"
alignment_valid=false
if python3 -c '
import json, sys
try:
    raise SystemExit(0 if json.load(sys.stdin).get("valid") else 1)
except Exception:
    raise SystemExit(1)
' <<<"$alignment_json"; then
    alignment_valid=true
fi

local_ready=false
fully_ready=false
for _ in $(seq 1 40); do
    if docker exec "$CONTAINER" bash -lc '
      source /opt/ros/humble/setup.bash
      source /root/go2_slam_ws/nav2_overlay/install/setup.bash
      check_active() {
        timeout 5 ros2 lifecycle get "$1" 2>/dev/null | grep -q "active \[3\]"
      }
      check_active /controller_server & p1=$!
      check_active /velocity_smoother & p2=$!
      check_active /collision_monitor & p3=$!
      rc=0
      wait "$p1" || rc=1
      wait "$p2" || rc=1
      wait "$p3" || rc=1
      exit "$rc"
    '; then
        local_ready=true
        # With no validated nav_map->odom TF the planner lifecycle transition
        # intentionally blocks inside the global costmap. Do not query that
        # transitioning node: the ROS CLI call itself waits indefinitely.
        if ! $alignment_valid; then
            break
        fi
        if docker exec "$CONTAINER" bash -lc '
          source /opt/ros/humble/setup.bash
          source /root/go2_slam_ws/nav2_overlay/install/setup.bash
          check_active() {
            timeout 5 ros2 lifecycle get "$1" 2>/dev/null | grep -q "active \[3\]"
          }
          check_active /planner_server & p1=$!
          check_active /bt_navigator & p2=$!
          rc=0
          wait "$p1" || rc=1
          wait "$p2" || rc=1
          exit "$rc"
        '; then
            fully_ready=true
            break
        fi
    fi
    sleep 0.5
done

if ! $local_ready; then
    echo "错误: Nav2 局部规划/安全节点未进入 active，请检查 $LOG" >&2
    docker logs "$CONTAINER" 2>&1 | tail -80 >&2 || true
    exit 1
fi
if $alignment_valid && ! $fully_ready; then
    echo "错误: 已有地图定位，但 Nav2 全局节点未进入 active，请检查 $LOG" >&2
    docker logs "$CONTAINER" 2>&1 | tail -80 >&2 || true
    exit 1
fi

echo "Nav2 服务已启动（Smac + SCAN 局部轨迹 + Nav2 跟踪/恢复/碰撞监控）"
echo "  SCAN 职责: 仅生成局部 B-spline，不直接输出底盘速度"
echo "  Nav2 安全速度: /go2/nav2/cmd_vel_safe"
if ! $fully_ready; then
    echo "  全局状态: 等待本次开机自动定位；定位成功后自动进入 active"
fi
echo "  日志: $LOG"
