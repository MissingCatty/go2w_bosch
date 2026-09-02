#!/bin/bash
# Build the pinned Humble Nav2 image and the isolated Humble overlay.
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
SCAN_WS="/home/unitree/scan_planner_ws"
IMAGE="${GO2_NAV2_IMAGE:-go2_nav2_humble:local}"

if [ ! -f "$SCAN_WS/install/setup.bash" ]; then
    echo "错误: SCAN-Planner 尚未构建: $SCAN_WS/install/setup.bash" >&2
    echo "请先运行 $SCAN_WS/build_go2w.sh" >&2
    exit 1
fi

if [ "${1:-}" = "--image" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "==> 构建 Nav2 Humble 镜像: $IMAGE"
    docker build -f "$WS/docker/nav2/Dockerfile" -t "$IMAGE" "$WS"
fi

echo "==> 构建 Nav2 Humble overlay"
docker run --rm --network=host --ipc=host \
    --volume="$WS:/root/go2_slam_ws" \
    --volume="$SCAN_WS:/root/scan_planner_ws:ro" \
    "$IMAGE" bash -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      source /root/scan_planner_ws/install/setup.bash
      cd /root/go2_slam_ws/nav2_overlay
      colcon build --symlink-install --event-handlers console_direct+
    '

echo "Nav2 overlay 构建完成: $WS/nav2_overlay/install"
