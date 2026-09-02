#!/bin/bash
set -euo pipefail

CONTAINER="go2_nav2_shadow"
UNIT="go2-nav2-shadow.service"

systemctl --user stop "$UNIT" >/dev/null 2>&1 || true
docker stop -t 5 "$CONTAINER" >/dev/null 2>&1 || true
systemctl --user reset-failed "$UNIT" >/dev/null 2>&1 || true
echo "Nav2 影子服务已停止；默认 SCAN 控制链不受影响"
