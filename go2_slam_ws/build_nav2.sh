#!/bin/bash
# Build the pinned Humble Nav2 image and the isolated Humble overlay.
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
IMAGE="${GO2_NAV2_IMAGE:-go2_nav2_humble:local}"

if [ "${1:-}" = "--image" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "==> 构建 Nav2 Humble 镜像: $IMAGE"
    docker build -f "$WS/docker/nav2/Dockerfile" -t "$IMAGE" "$WS"
fi

echo "==> 构建 Nav2 Humble overlay"
docker run --rm --network=host --ipc=host \
    --volume="$WS:/root/go2_slam_ws" \
    "$IMAGE" bash -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      cd /root/go2_slam_ws/nav2_overlay
      colcon build --symlink-install --event-handlers console_direct+
    '

echo "Nav2 overlay 构建完成: $WS/nav2_overlay/install"
