#!/bin/bash
# Restore the XT16 + LIO-SAM localization backend before navigation starts.
# Point-LIO is deliberately mapping-only and must never feed SCAN-Planner.
set -euo pipefail

SLAM_WS="${GO2_SLAM_WS:-/home/unitree/go2_slam_ws}"

if systemctl --user is-active --quiet go2-front-pointlio.service; then
    "$SLAM_WS/stop_front_lidar_mapping.sh"
fi

if ! systemctl --user is-active --quiet go2-xt16.service || \
   ! systemctl --user is-active --quiet go2-lio-sam.service || \
   ! docker inspect -f '{{.State.Running}}' dddmr_humble 2>/dev/null | grep -q true; then
    "$SLAM_WS/start.sh"
fi

if systemctl --user is-active --quiet go2-front-pointlio.service; then
    echo "错误: 前置雷达 Point-LIO 仍在运行，拒绝进入导航" >&2
    exit 1
fi

for unit in go2-xt16.service go2-lio-sam.service; do
    if ! systemctl --user is-active --quiet "$unit"; then
        echo "错误: 导航定位后端未就绪: $unit" >&2
        exit 1
    fi
done

for _ in $(seq 1 30); do
    status="$(curl -fsS --max-time 2 http://127.0.0.1:8890/api/status 2>/dev/null || true)"
    if python3 -c '
import json, sys
try:
    health = json.loads(sys.stdin.read()).get("health", {})
except Exception:
    raise SystemExit(1)
raise SystemExit(0 if health.get("pose_online") and health.get("pose_valid") else 1)
' <<<"$status"; then
        echo "导航定位后端已恢复为 XT16 + LIO-SAM"
        exit 0
    fi
    sleep 1
done

echo "错误: LIO-SAM 已启动，但网页未收到有效位姿" >&2
exit 1
