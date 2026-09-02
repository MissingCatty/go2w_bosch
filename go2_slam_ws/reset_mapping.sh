#!/bin/bash
# Discard the current in-memory LIO-SAM map and start a fresh mapping session.
# Saved NPZ maps are intentionally untouched.  The Web API owns user confirmation
# and the chassis gate; this script only replaces the LIO container.
set -euo pipefail

if systemctl --user is-active --quiet go2-front-pointlio.service; then
    exec /home/unitree/go2_slam_ws/reset_front_lidar_mapping.sh
fi

DDDMR_RUN="/home/unitree/dddmr_navigation/run_xt16_mapping.sh"
LIO_UNIT="go2-lio-sam.service"
LIO_CONTAINER="dddmr_humble"
LIO_LOG="/tmp/dddmr_lio_$(id -u).log"

systemctl --user --no-block stop "$LIO_UNIT" >/dev/null 2>&1 || true
docker stop -t 10 "$LIO_CONTAINER" >/dev/null 2>&1 || true
systemctl --user reset-failed "$LIO_UNIT" >/dev/null 2>&1 || true

# A transient --collect unit must disappear before systemd-run may reuse its name.
for _ in $(seq 1 60); do
    state="$(systemctl --user show -p LoadState --value "$LIO_UNIT" 2>/dev/null || true)"
    [ "$state" = "not-found" ] && break
    sleep 0.1
done
if [ "$(systemctl --user show -p LoadState --value "$LIO_UNIT" 2>/dev/null || true)" != "not-found" ]; then
    echo "LIO-SAM 服务未能停止" >&2
    exit 1
fi

docker rm "$LIO_CONTAINER" >/dev/null 2>&1 || true
: >"$LIO_LOG"
systemd-run --user --unit=go2-lio-sam --collect \
    --property=Restart=on-failure --property=RestartSec=3 \
    --property=TimeoutStopSec=5 \
    --property="StandardOutput=append:$LIO_LOG" \
    --property="StandardError=append:$LIO_LOG" \
    /bin/bash "$DDDMR_RUN" lio >/dev/null

for _ in $(seq 1 100); do
    if docker inspect -f '{{.State.Running}}' "$LIO_CONTAINER" 2>/dev/null | grep -q true; then
        echo "LIO-SAM 当前建图已清除，正在重新初始化"
        exit 0
    fi
    sleep 0.1
done

echo "LIO-SAM 容器启动失败，请检查 $LIO_LOG" >&2
exit 1
