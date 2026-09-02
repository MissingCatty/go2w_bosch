# SCAN-Planner × GO2-W

这个工作区提供 SCAN-Planner 的 ROS 2 Humble 移植和 GO2-W/LIO-SAM
适配。当前默认以 `planner_only:=true` 运行：SCAN 只生成局部无碰 B 样条，
不再直接控制底盘，也不再启动旧版闭环控制器和独立脱困节点。

## 当前导航链路

```text
LIO-SAM odom + 当前点云
  -> Nav2 Smac 生成全局路径
  -> go2_scan_nav2_controller 把全局路径送给 SCAN
  -> SCAN 实时三维占据 + 局部 B 样条
  -> Nav2 controller_server 跟踪 B 样条
  -> Nav2 velocity_smoother
  -> Nav2 collision_monitor
  -> go2_chassis_safety_gate（默认锁定）
  -> Unitree Sport API
```

Nav2 负责目标生命周期、全局规划、进度判断、行为树恢复、速度平滑和碰撞
监控；SCAN 只负责当前激光视野内的局部轨迹生成。SCAN 输出的原生消息是
`/scan_planner/planning/bspline`，由 Nav2 控制器插件消费。

## 输入与坐标系

```text
/lio_sam/mapping/odometry
  -> lio_pose_adapter
     -> /scan_planner/sensor_pose
     -> /scan_planner/body_pose

/lio_sam/deskew/cloud_deskewed + poses
  -> SCAN 实时三维局部占据

Nav2 Smac path
  -> /scan_planner/global_path
  -> SCAN local B-spline
```

适配器应用已标定的 `base_link -> rslidar` 变换
`(0.1701, 0, 0.0908, yaw=+90deg)`。`world_z_offset=0.53m` 用于将
LIO 启动原点与保存地图的地面高度对齐，可在
`src/go2_scan_planner_bridge/config/go2w.yaml` 中配置。

保存地图的 `map -> odom` 变换只有在 Web 自动定位对当前开机和当前地图验证
通过后才会发布。静态地图供 Nav2 Smac 做全局规划；SCAN 的占据、碰撞检查和
局部绕行只使用实时激光数据。

## 构建与启动

```bash
cd /home/unitree/scan_planner_ws
./build_go2w.sh
./start_go2w_dry.sh
```

`start_go2w_dry.sh` 会显式传入 `planner_only:=true`。需要临时复现历史独立
控制链时才可手工传入 `planner_only:=false`；正常整机导航不要这样做。

完整系统从主工作区启动：

```bash
cd /home/unitree/go2_slam_ws
./start_navigation.sh
```

## 安全边界

- SCAN 的 `/scan_planner/cmd_vel_test` 在默认模式下没有发布者。
- 只有 Nav2 的 `cmd_vel` 链能够进入主机侧底盘安全门。
- Web 取消导航会停止 Nav2 任务、锁定安全门并发送停止脉冲。
- 启动后必须先完成地图定位，再由操作者手工解锁真实底盘。
- 不要把任何 SCAN 或 Nav2 中间速度话题直接接到 Sport API。

控制器插件和完整边界说明见
`/home/unitree/go2_slam_ws/NAV2_GO2W_INTEGRATION.md`。
