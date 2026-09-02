#!/bin/bash
# GO2-W LIO-SAM 全链路一键启动：XT16 -> LowState IMU -> LIO-SAM -> Web。
set -eo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
BIN="$WS/module/unitree_slam/bin"
LIBDIR="$WS/module/unitree_slam/lib"
DDDMR_WS="/home/unitree/dddmr_navigation"
DDDMR_RUN="$DDDMR_WS/run_xt16_mapping.sh"
BUILD=false
[ "${1:-}" = "--build" ] && BUILD=true
RUN_TAG="${UID:-$(id -u)}"
XT16_LOG="/tmp/xt16_driver_${RUN_TAG}.log"
HOST_LOG="/tmp/go2_slam_launch_${RUN_TAG}.log"
LIO_LOG="/tmp/dddmr_lio_${RUN_TAG}.log"
HOST_RESTARTED=false
LIDAR_RESTARTED=false

source "$WS/setup_env.sh"

topic_has_data() {
    local topic="$1"
    # PointCloud2/Imu 都有 header；--no-arr 避免打印点云大数组。关闭本子 shell 的
    # pipefail，因为 grep 命中退出后 ros2 echo 收到 SIGPIPE 属于正常结束。
    (set +o pipefail
     PYTHONUNBUFFERED=1 timeout 3 ros2 topic echo "$topic" --no-arr --no-str 2>/dev/null | \
         grep -m1 -q '^header:')
}

wait_topic_data() {
    local topic="$1" attempts="${2:-10}" i
    for ((i = 0; i < attempts; i++)); do
        topic_has_data "$topic" && return 0
        sleep 1
    done
    return 1
}

stop_user_unit() {
    local unit="$1" i load_state
    systemctl --user --no-block stop "$unit" 2>/dev/null || true
    systemctl --user reset-failed "$unit" 2>/dev/null || true
    for ((i = 0; i < 50; i++)); do
        load_state="$(systemctl --user show -p LoadState --value "$unit" 2>/dev/null || true)"
        [ "$load_state" = "not-found" ] && return 0
        sleep 0.1
    done
}

stop_lio_service() {
    # docker run 是前台客户端：先让 systemd 不再重启，再由 Docker API 停容器。
    systemctl --user --no-block stop go2-lio-sam.service 2>/dev/null || true
    docker stop -t 10 dddmr_humble >/dev/null 2>&1 || true
    stop_user_unit go2-lio-sam.service
}

# 精确清理同名遗留进程。旧版曾以 root 启动，普通 pkill 无权停止时，使用已有
# Docker 镜像进入 host PID namespace，只向已解析出的具体 PID 发送 TERM。
stop_exact_process() {
    local name="$1" pids
    pkill -TERM -x "$name" 2>/dev/null || true
    sleep 1
    pids="$(pgrep -x "$name" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        docker run --rm --pid=host --privileged dddmr_humble:nav \
            bash -lc "kill -TERM $pids" >/dev/null 2>&1 || true
        sleep 2
    fi
}

echo "==> [1/5] USB3.0 供电激活（雷达）"
if [ "$(cat /sys/class/gpio/PP.06/value 2>/dev/null || true)" != "1" ]; then
    busybox devmem 0x02430030 w 0x004 2>/dev/null || true
    echo 446 > /sys/class/gpio/export 2>/dev/null || true
    echo out > /sys/class/gpio/PP.06/direction 2>/dev/null || true
    echo 1 > /sys/class/gpio/PP.06/value 2>/dev/null || true
fi

echo "==> [2/5] 构建 host 工作空间"
if [ ! -d "$WS/install/go2_slam_core" ] || \
   [ ! -d "$WS/install/go2_imu_bridge" ] || \
   [ ! -d "$WS/install/go2_remembr" ] || \
   [ ! -d "$WS/install/go2_slam_web" ] || $BUILD; then
    # The Nav2 Humble overlay lives outside src/ because this host workspace is
    # ROS 2 Foxy. Restrict host discovery so a normal rebuild never tries to
    # resolve Humble-only nav2_msgs dependencies.
    (cd "$WS" && colcon build --base-paths "$WS/src" --symlink-install)
else
    echo "    已构建（强制重编: ./start.sh --build）"
fi
source "$WS/install/setup.bash"

if $BUILD; then
    echo "==> [3/5] 构建 Humble LIO-SAM"
    # 先关监管服务，避免 docker stop 被 Restart=on-failure 立即拉起。
    stop_lio_service
    bash "$DDDMR_RUN" build
else
    echo "==> [3/5] Humble LIO-SAM 使用现有构建"
fi

echo "==> [4/5] 启动并检查 XT16、IMU 桥与 Web"
if ! topic_has_data /unitree/slam_lidar/points; then
    stop_user_unit go2-xt16.service
    stop_exact_process xt16_driver
    : >"$XT16_LOG"
    systemd-run --user --unit=go2-xt16 --collect \
        --property=Restart=on-failure --property=RestartSec=2 \
        --property="StandardOutput=append:$XT16_LOG" \
        --property="StandardError=append:$XT16_LOG" \
        --working-directory="$BIN" \
        --setenv="LD_LIBRARY_PATH=$LIBDIR:/usr/local/lib" \
        "$BIN/xt16_driver" >/dev/null
    LIDAR_RESTARTED=true
fi
if ! wait_topic_data /unitree/slam_lidar/points 20; then
    echo "错误: XT16 点云未就绪，请检查 $XT16_LOG" >&2
    exit 1
fi

if $BUILD || \
   ! systemctl --user is-active --quiet go2-slam-host.service || \
   ! topic_has_data /dog_imu_lio || \
   ! curl -fsS --max-time 1 http://127.0.0.1:8890/api/status >/dev/null; then
    stop_user_unit go2-slam-host.service
    pkill -f '/opt/ros/foxy/bin/ros2 launch go2_slam_bringup slam.launch.py' 2>/dev/null || true
    pkill -x web_server 2>/dev/null || true
    pkill -f '/go2_slam_core/imu_attitude_bridge' 2>/dev/null || true
    pkill -f '/go2_imu_bridge/imu_attitude_bridge' 2>/dev/null || true
    pkill -f '/go2_imu_bridge/lidar_preview_bridge' 2>/dev/null || true
    sleep 1
    : >"$HOST_LOG"
    systemd-run --user --unit=go2-slam-host --collect \
        --property=Restart=on-failure --property=RestartSec=2 \
        --property="StandardOutput=append:$HOST_LOG" \
        --property="StandardError=append:$HOST_LOG" \
        /bin/bash -c "source '$WS/setup_env.sh'; export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1; exec ros2 launch go2_slam_bringup slam.launch.py" \
        >/dev/null
    HOST_RESTARTED=true
fi
if ! wait_topic_data /dog_imu_lio 10 || \
   ! curl -fsS --retry 10 --retry-delay 1 --retry-connrefused --max-time 2 \
        http://127.0.0.1:8890/api/status >/dev/null; then
    echo "错误: IMU 桥或 Web 未就绪，请检查 $HOST_LOG" >&2
    exit 1
fi

echo "==> [5/5] 启动 Humble LIO-SAM 容器"
if $HOST_RESTARTED || $LIDAR_RESTARTED || \
   ! docker inspect -f '{{.State.Running}}' dddmr_humble 2>/dev/null | grep -q true; then
    stop_lio_service
    docker rm dddmr_humble >/dev/null 2>&1 || true
    : >"$LIO_LOG"
    systemd-run --user --unit=go2-lio-sam --collect \
        --property=Restart=on-failure --property=RestartSec=3 \
        --property=TimeoutStopSec=5 \
        --property="StandardOutput=append:$LIO_LOG" \
        --property="StandardError=append:$LIO_LOG" \
        /bin/bash "$DDDMR_RUN" lio >/dev/null
fi

for _ in $(seq 1 20); do
    if docker inspect -f '{{.State.Running}}' dddmr_humble 2>/dev/null | grep -q true; then
        break
    fi
    sleep 1
done
if ! docker inspect -f '{{.State.Running}}' dddmr_humble 2>/dev/null | grep -q true; then
    echo "错误: LIO-SAM 容器未启动，请检查 $LIO_LOG" >&2
    exit 1
fi

sleep 8
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "============================================================"
echo "  控制台: http://${IP}:8890"
curl -fsS --max-time 3 http://127.0.0.1:8890/api/status || \
    echo "  Web 状态暂不可读，请检查 $HOST_LOG"
echo
echo "  日志: $XT16_LOG  $HOST_LOG  $LIO_LOG"
echo "============================================================"

# ros2 CLI discovery starts a background daemon during the readiness probes.
# Runtime nodes communicate directly through DDS and do not need it; leaving
# the daemon alive on this dense graph costs roughly one third of a CPU core.
ros2 daemon stop >/dev/null 2>&1 || true
