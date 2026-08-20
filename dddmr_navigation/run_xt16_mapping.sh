#!/bin/bash
# 进入 dddmr 容器(host 网络 + CycloneDDS 与机器人 Foxy 互通)
# 容器内已 colcon build 过时: bash run_xt16_mapping.sh build|run|lio|localize|navigation|shell

IMG=${DDDMR_IMG:-dddmr_humble:nav}   # 默认用已装好 sport 桥依赖的 commit 镜像
REPO=/home/unitree/dddmr_navigation
CYCLONE=/home/unitree/cyclonedds_ws/cyclonedds.xml

docker_run() {
  if [ -t 0 ]; then TTY="-it"; else TTY="-i"; fi
  docker run ${TTY} --rm \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${REPO}:/root/dddmr_navigation" \
    --volume="${CYCLONE}:/root/cyclonedds.xml" \
    --name="dddmr_humble" \
    ${IMG} "$@"
}

case "$1" in
  build)
    docker_run bash -c 'set -o pipefail; source /opt/ros/humble/setup.bash && cd /root/dddmr_navigation && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=OFF 2>&1 | tail -40'
    ;;
  run)
    docker_run bash -c 'mkdir -p /root/dddmr_maps && source /opt/ros/humble/setup.bash && source /root/dddmr_navigation/install/setup.bash && export CYCLONEDDS_URI=file:///root/cyclonedds.xml && export DDDMR_MAPPING_DIR=/root/dddmr_maps/ && ros2 launch lego_loam_bor xt16_mapping.launch'
    ;;
  lio)
    docker_run bash -c 'mkdir -p /root/dddmr_maps && source /opt/ros/humble/setup.bash && source /root/dddmr_navigation/install/setup.bash && export CYCLONEDDS_URI=file:///root/cyclonedds.xml && ros2 launch lio_sam lio_xt16.launch.py'
    ;;
  localize)
    docker_run bash -c 'mkdir -p /root/dddmr_maps && source /opt/ros/humble/setup.bash && source /root/dddmr_navigation/install/setup.bash && export CYCLONEDDS_URI=file:///root/cyclonedds.xml && (python3 /root/dddmr_navigation/odom_bridge.py &) && sleep 2 && ros2 launch lego_loam_bor xt16_localization.launch'
    ;;
  navigation)
    docker_run bash -c 'mkdir -p /root/dddmr_maps && source /opt/ros/humble/setup.bash && source /root/dddmr_navigation/install/setup.bash && export CYCLONEDDS_HOME=/opt/cdds_home && export LD_LIBRARY_PATH=/opt/cdds_home/lib:/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH && (python3 /root/dddmr_navigation/odom_bridge.py &) && (sleep 3 && python3 /root/dddmr_navigation/cmd_vel_sport_bridge.py &) && sleep 2 && ros2 launch lego_loam_bor xt16_navigation.launch'
    ;;
  shell)
    docker_run bash
    ;;
  *)
    echo "用法: $0 build|run|lio|localize|navigation|shell"
    ;;
esac
