#!/bin/bash
# Start SCAN-Planner against the live GO2-W LIO graph with actuator output isolated.
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
NAV_DIR="/home/unitree/go2_slam_ws/maps/navigation"
IMAGE="${SCAN_PLANNER_IMAGE:-dddmr_humble:nav}"
CONTAINER="go2_scan_planner_dry"
UNIT="go2-scan-planner-dry.service"
LOG="/tmp/go2_scan_planner_dry_$(id -u).log"

if [ "${1:-}" = "--build" ] || [ ! -x "$WS/build/scan_planner/scan_planner_node" ]; then
  "$WS/build_go2w.sh"
fi

for map_file in \
  map_20260811_155640_273_nav.yaml \
  map_20260811_155640_273_nav_inflated.yaml \
  map_20260811_155640_273_nav_inflated.pgm \
  map_20260811_155640_273_nav_obstacles.pcd \
  map_20260811_155640_273_nav.json; do
  if [ ! -r "$NAV_DIR/$map_file" ]; then
    echo "错误: 导航地图缺失: $NAV_DIR/$map_file" >&2
    echo "请先运行 ~/go2_slam_ws/tools/prepare_navigation_map.py" >&2
    exit 1
  fi
done

status="$(curl -fsS --max-time 3 http://127.0.0.1:8890/api/status || true)"
# 导航页会主动取消大地图订阅以节省带宽，因此这里只要求实时 LIO 位姿健康；
# 全局 A* 由 Web 使用已保存地图；SCAN 用实时雷达绕临时障碍，同时把
# 膨胀后的静态自由区作为不可穿越的边界。
if ! python3 -c 'import json,sys; h=json.loads(sys.stdin.read()).get("health",{}); sys.exit(0 if h.get("pose_online") and h.get("pose_valid") else 1)' <<<"$status"; then
  echo "错误: LIO-SAM 位姿未就绪，请先运行 ~/go2_slam_ws/start.sh" >&2
  exit 1
fi

systemctl --user stop "$UNIT" >/dev/null 2>&1 || true
docker stop -t 5 "$CONTAINER" >/dev/null 2>&1 || true
systemctl --user reset-failed "$UNIT" >/dev/null 2>&1 || true
for _ in $(seq 1 30); do
  load_state="$(systemctl --user show "$UNIT" -p LoadState --value 2>/dev/null || true)"
  if [ -z "$load_state" ] || [ "$load_state" = "not-found" ]; then
    break
  fi
  sleep 0.1
done
load_state="$(systemctl --user show "$UNIT" -p LoadState --value 2>/dev/null || true)"
if [ -n "$load_state" ] && [ "$load_state" != "not-found" ]; then
  echo "错误: 旧的 $UNIT 未能及时释放" >&2
  exit 1
fi
: >"$LOG"

# --ipc=host is required because LIO-SAM and SCAN use Fast DDS shared memory.
systemd-run --user --unit="${UNIT%.service}" --collect \
  --property=Restart=on-failure --property=RestartSec=2 \
  --property=TimeoutStopSec=8 \
  --property="StandardOutput=append:$LOG" \
  --property="StandardError=append:$LOG" \
  docker run --rm --name "$CONTAINER" --network=host --ipc=host \
    --env=PYTHONDONTWRITEBYTECODE=1 \
    --volume="$WS:/root/scan_planner_ws" \
    --volume="$NAV_DIR:/root/go2_navigation_maps:ro" \
    "$IMAGE" bash -lc '
      source /opt/ros/humble/setup.bash
      source /root/scan_planner_ws/install/setup.bash
      exec ros2 launch go2_scan_planner_bridge go2w_scan_planner.launch.py dry_run:=true
    ' >/dev/null

for _ in $(seq 1 30); do
  logs="$(docker logs "$CONTAINER" 2>&1 || true)"
  if grep -q 'body=' <<<"$logs" && \
     grep -q 'static navigation map loaded' <<<"$logs" && \
     grep -q 'loaded immutable inflated navigation grid' <<<"$logs"; then
    break
  fi
  sleep 0.5
done
logs="$(docker logs "$CONTAINER" 2>&1 || true)"
if ! grep -q 'body=' <<<"$logs" || \
   ! grep -q 'static navigation map loaded' <<<"$logs" || \
   ! grep -q 'loaded immutable inflated navigation grid' <<<"$logs"; then
  echo "错误: SCAN-Planner 位姿或导航地图服务未就绪，请检查 $LOG" >&2
  exit 1
fi

echo "SCAN-Planner 干跑模式已启动"
echo "  全局规划: Web A* 使用保存的静态导航图"
echo "  局部规划: 实时点云绕行 + 静态膨胀栅格防穿墙"
echo "  碰撞体: SCAN 双圆柱模型（不将静态 3D 点混入实时占据记忆）"
echo "  控制输出: /scan_planner/cmd_vel_test（仅经默认锁定的安全门转发）"
echo "  发送前方测试目标: $WS/send_forward_test_goal.sh 1.0"
echo "  日志: $LOG"
