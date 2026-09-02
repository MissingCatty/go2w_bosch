# GO2-W 前置雷达建图

这套试验链路使用 GO2-W 自带前置 Unitree 雷达，而不是顶部 XT16：

`/utlidar/cloud + /utlidar/imu (ROS 2) -> ros1_bridge -> Point-LIO (ROS 1) -> Web (ROS 2)`

前置雷达的点云和它自己的 IMU 使用同一套设备时钟，不能把点云与机身
`/dog_imu_lio` 混用。实机测得点云约 15 Hz、每帧约 3100–3900 点；所有点的
`ring` 都为 1，所以原有按多线 ring 组织点云的 LIO-SAM 不适合直接使用。

运行目录：

- Point-LIO: `/home/unitree/front_lidar_pointlio_ws`，基于 Unitree
  `point_lio_unilidar` commit `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91`
- ROS 1 bridge: `/home/unitree/front_lidar_bridge_ws`，基于 ROS 2 Foxy
  `ros1_bridge` commit `689a932499befbd1ec3cb273a1054430e55a43c3`

使用：

```bash
/home/unitree/go2_slam_ws/start_front_lidar_mapping.sh
/home/unitree/go2_slam_ws/stop_front_lidar_mapping.sh
```

启动脚本会先锁定底盘、停止 SCAN/XT16/LIO-SAM，再按 `roscore -> bridge ->
Point-LIO` 顺序启动，并将网页预览切到前置雷达。网页仍保持“建图默认关闭”，
需要操作者点击“开始建图”。清除按钮会重启 Point-LIO，只丢弃未保存地图。

Point-LIO 的 GO2-W 修改保留在上述独立 Git 工作区中，可分别用 `git diff`
查看；主要包括前置雷达话题/频率、每秒累计地图发布、网页标准输出话题和关闭
重复 PCD 缓存。桥接工作区只编译建图需要的标准消息类型，避免 ARM 机器内存不足。
