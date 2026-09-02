#!/bin/bash
# GO2-W 重启后一键拉起完整导航链路。
#
# 顺序：XT16 -> 内置 IMU/Web -> LIO-SAM -> SCAN-Planner -> Nav2 影子 -> 端侧 VLM。
# 该脚本绝不启用真实底盘；Linux 重启后仍需在网页完成自动定位，再由操作者
# 显式点击“启用真实底盘”。重复执行时，如果底盘已经启用会直接拒绝操作。
set -euo pipefail

SLAM_WS="$(cd "$(dirname "$0")" && pwd)"
SCAN_WS="/home/unitree/scan_planner_ws"
WEB_URL="${GO2_WEB_URL:-http://127.0.0.1:8890}"
BUILD=false

usage() {
    echo "用法: $0 [--build]"
    echo "  --build  启动前重编 host、LIO-SAM 和 SCAN-Planner"
}

case "${1:-}" in
    "") ;;
    --build) BUILD=true ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

post_json() {
    local path="$1" body="$2"
    curl -fsS --max-time 5 -H 'Content-Type: application/json' \
        --data "$body" "$WEB_URL$path"
}

lock_chassis() {
    # Both endpoints are intentional: chassis=false closes the actuator gate;
    # cancel also clears any stale SCAN reference path and publishes zero speed.
    post_json /api/navigation/chassis '{"enabled":false}' >/dev/null 2>&1 || true
    post_json /api/navigation/cancel '{}' >/dev/null 2>&1 || true
}

fail_safe() {
    local code=$?
    trap - ERR
    set +e
    lock_chassis
    echo >&2
    echo "启动失败；底盘保持锁定。请检查：" >&2
    echo "  /tmp/xt16_driver_$(id -u).log" >&2
    echo "  /tmp/go2_slam_launch_$(id -u).log" >&2
    echo "  /tmp/dddmr_lio_$(id -u).log" >&2
    echo "  /tmp/go2_scan_planner_dry_$(id -u).log" >&2
    exit "$code"
}
trap fail_safe ERR

# Never tear down a live controller underneath a moving robot.
if chassis_json="$(curl -fsS --max-time 2 "$WEB_URL/api/navigation/chassis" 2>/dev/null)"; then
    if python3 -c \
        'import json,sys; sys.exit(0 if json.load(sys.stdin).get("enabled") else 1)' \
        <<<"$chassis_json"; then
        echo "错误: 真实底盘当前已启用。请先在网页取消导航并锁定底盘。" >&2
        exit 1
    fi
fi

echo "==> [1/8] 清理未执行任务的旧 SCAN 实例"
"$SCAN_WS/stop_go2w.sh" >/dev/null 2>&1 || true

echo "==> [2/8] 拉起 XT16、内置 IMU/Web 和 LIO-SAM"
if $BUILD; then
    "$SLAM_WS/start.sh" --build
else
    "$SLAM_WS/start.sh"
fi

echo "==> [3/8] 设置并确认狗本机音量为 10%"
/bin/bash -c \
    "source '$SLAM_WS/setup_env.sh' >/dev/null 2>&1; exec python3 '$SLAM_WS/tools/set_go2_volume.py' --percent 10"

echo "==> [4/8] 切换 Web 到导航模式"
mode_json="$(post_json /api/mode '{"mode":"navigation"}')"
python3 -c '
import json, sys
result = json.load(sys.stdin)
if not result.get("success") or result.get("mode") != "navigation":
    raise SystemExit("Web 切换导航模式失败: " + str(result.get("message", "未知错误")))
' <<<"$mode_json"

echo "==> [5/8] 拉起并检查 SCAN-Planner"
if $BUILD; then
    "$SCAN_WS/start_go2w_dry.sh" --build
else
    "$SCAN_WS/start_go2w_dry.sh"
fi

echo "==> [6/8] 拉起 Nav2 影子规划链"
if $BUILD; then
    "$SLAM_WS/build_nav2.sh"
fi
"$SLAM_WS/start_nav2_shadow.sh"

echo "==> [7/8] 拉起端侧 Qwen3.5 视觉语言模型"
vlm_ready=false
if $BUILD; then
    "$SLAM_WS/stop_remembr_vlm.sh" >/dev/null 2>&1 || true
    if ! "$SLAM_WS/build_remembr_vlm.sh"; then
        echo "警告: VLM 编译失败，SLAM/导航继续保持可用" >&2
    fi
fi
if "$SLAM_WS/start_remembr_vlm.sh"; then
    vlm_ready=true
else
    echo "警告: VLM 未就绪，语义记忆将安全降级，SLAM/导航不受影响" >&2
fi

echo "==> [8/8] 清除旧目标、锁定底盘并做总体验收"
lock_chassis
sleep 1

units=(
    go2-xt16.service
    go2-slam-host.service
    go2-lio-sam.service
    go2-scan-planner-dry.service
    go2-nav2-shadow.service
)
for unit in "${units[@]}"; do
    if ! systemctl --user is-active --quiet "$unit"; then
        echo "错误: $unit 未运行" >&2
        exit 1
    fi
done
if $vlm_ready && ! systemctl --user is-active --quiet go2-remembr-vlm.service; then
    echo "错误: go2-remembr-vlm.service 未运行" >&2
    exit 1
fi

status_json="$(curl -fsS --max-time 5 "$WEB_URL/api/status")"
chassis_json="$(curl -fsS --max-time 5 "$WEB_URL/api/navigation/chassis")"
alignment_json="$(curl -fsS --max-time 5 "$WEB_URL/api/navigation/alignment")"
python3 - "$status_json" "$chassis_json" "$alignment_json" <<'PY'
import json
import sys

status, chassis, alignment = (json.loads(value) for value in sys.argv[1:])
health = status.get('health', {})
failures = []
if status.get('mode') != 'navigation':
    failures.append('Web 未处于导航模式')
if not health.get('pose_online') or not health.get('pose_valid'):
    failures.append('LIO-SAM 导航位姿未就绪')
if not chassis.get('connected'):
    failures.append('底盘 Sport API 未连接')
if chassis.get('enabled'):
    failures.append('底盘未保持锁定')
if any(abs(float(value)) > 1e-6 for value in chassis.get('output', [])):
    failures.append('底盘输出不是零速度')
if failures:
    raise SystemExit('；'.join(failures))

print('    LIO 位姿: 在线且有效')
print('    SCAN/底盘门: 已连接，零速度，已锁定')
if alignment.get('valid'):
    pose = alignment.get('map_pose', {})
    print('    地图定位: 有效 (%.2f, %.2f, %.1f°)' % (
        float(pose.get('x', 0.0)), float(pose.get('y', 0.0)),
        float(pose.get('yaw_deg', 0.0))))
else:
    print('    地图定位: 待自动定位（重启后的正常安全状态）')
PY

ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "============================================================"
echo "完整导航服务已拉起，真实底盘保持锁定"
echo "狗本机音量: 10%"
echo "端侧语义记忆: $($vlm_ready && echo 'Qwen3.5 已就绪' || echo '安全降级')"
echo "Nav2: Smac + MPPI 影子模式（真实控制仍为 SCAN）"
echo "控制台: http://${ip:-127.0.0.1}:8890"
echo "下一步: 在导航页完成重启后自动定位，再手动启用真实底盘"
echo "============================================================"
