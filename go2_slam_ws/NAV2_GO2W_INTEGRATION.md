# GO2-W Nav2 + SCAN 局部轨迹集成

本工作空间不更换宇树底层系统、LIO-SAM、控制协议或 JetPack。Nav2 运行在独立
ROS 2 Humble 容器中，SCAN-Planner 只生成实时局部 B-spline；任务、全局规划、
轨迹跟踪、恢复行为、速度生命周期和最终碰撞停车均由 Nav2 管理。

## 运行架构

```text
Web 目标（nav_map）
  -> Nav2 NavigateToPose / BT
  -> Smac State Lattice 全局 Path（nav_map）
  -> ScanTrajectoryController
       -> TF 转为 odom 后发布 /scan_planner/global_path
       -> SCAN 实时点云局部规划
       <- /scan_planner/planning/bspline
       -> Nav2 内部 B-spline 位姿/航向闭环
  -> /go2/nav2/cmd_vel_raw
  -> Nav2 Velocity Smoother
  -> Nav2 Collision Monitor
  -> /go2/nav2/cmd_vel_safe
  -> go2_chassis_safety_gate
  -> Unitree Sport API
```

SCAN 的 `closed_loop_controller` 和 `realtime_recovery.py` 在默认启动中不再运行，
`/scan_planner/cmd_vel_test` 没有发布者。新插件位于
`nav2_overlay/src/go2_scan_nav2_controller`，直接解析 SCAN 原生
`scan_planner_msgs/msg/Bspline`，不会用 MPPI 对局部轨迹做第二次优化。

## 职责边界

| 环节 | 当前实现 |
|---|---|
| 全局规划 | Nav2 Smac State Lattice；5 cm、0.5 m 最小转弯半径、正常路径禁止倒车 |
| 局部轨迹 | SCAN；3 m 局部目标、实时三维点云、曲率约束 B-spline |
| 轨迹跟踪 | Nav2 Controller 插件；20 Hz、位置/航向反馈、曲率前馈、GO2-W 横移 |
| 速度范围 | 前进 0.50 m/s、横移 ±0.10 m/s、角速度 ±0.45 rad/s |
| 目标判定 | Nav2 Goal Checker；XY 0.30 m，SCAN 内部停止阈值同步为 0.30 m |
| 动态安全 | SCAN 局部避障 + Nav2 costmap + Collision Monitor |
| 恢复 | Nav2 清 costmap、等待、后退 0.20 m、旋转 45°，总次数有界 |
| 最终控制 | 默认锁定的 Foxy `go2_chassis_safety_gate` |

地图定位仍由 Web 自动定位负责。定位有效时发布 `nav_map -> odom`；未标定时
Nav2 局部安全节点可以运行，但全局 Planner/BT 等待 TF，安全门独立拒绝运动。

## 失效与代际处理

- 每个 Nav2 `setPlan()` 都建立新轨迹代际；早于新全局路径的 SCAN B-spline 会被拒绝。
- Nav2 停止调用 Controller 超过 0.60 s 后，插件在本地作废轨迹，旧速度不能复用。
- 首条局部轨迹超过 2 s 未生成，或局部无路持续 3 s，Controller 向 Nav2 报失败。
- SCAN 暂时无路时先发布零速；持续失败后由 Nav2 BT 执行有界恢复和全局重规划。
- 跟踪误差超过 0.50 m 时请求 SCAN 从当前里程计状态重新生成局部轨迹。
- 全局路径只在新目标或失效时重算，不再用固定 5 s 刷新打断 SCAN。
- Velocity Smoother 使用 `OPEN_LOOP` 连续爬升。GO2-W 的 LIO 在轮子克服静摩擦
  前会报告零速度；若在这里使用 `CLOSED_LOOP`，输出会永久卡在单周期增量。
- 行为树恢复复用旧 Smac 路径时，插件使用最新的固定 `nav_map -> odom`
  标定，避免旧路径时间戳跌出 TF 缓存。
- Collision Monitor 和安全门仍分别执行点云、里程计、姿态、心跳与命令超时停车。

## 构建与启动

Nav2 插件依赖 SCAN 的消息包，因此先构建 SCAN：

```bash
cd /home/unitree/scan_planner_ws
./build_go2w.sh

cd /home/unitree/go2_slam_ws
./build_nav2.sh
./start_nav2_shadow.sh
```

完整链路仍可使用：

```bash
./start_navigation.sh --build   # 源码/参数变化后
./start_navigation.sh           # 普通启动
```

脚本不会自动启用真实底盘。进入 Web 导航页完成本次开机的地图定位后，操作者
才能启用底盘并发送目标。默认控制后端为 `nav2`；这里的 Nav2 Controller 内部
已经使用 SCAN 生成局部轨迹，不再是两个并行控制后端。

检查运行边界：

```bash
docker exec go2_nav2_shadow bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/scan_planner_ws/install/setup.bash
  source /root/go2_slam_ws/nav2_overlay/install/setup.bash
  ros2 lifecycle get /controller_server
  ros2 param get /controller_server FollowPath.plugin
  ros2 topic info /scan_planner/planning/bspline -v
  ros2 topic info /scan_planner/cmd_vel_test
'
```

预期插件为 `go2_scan_nav2_controller/ScanTrajectoryController`，B-spline 有一个
`controller_server` 订阅者，而 `/scan_planner/cmd_vel_test` 的发布者数量为 0。

## 已完成的锁底盘验收

- Humble 插件编译和 B-spline 数学单元测试通过。
- Controller 插件成功装载并进入 `active`。
- 锁底盘 FollowPath 测试中，SCAN 发布新 B-spline，插件发布局部 Path。
- `cmd_vel_raw -> cmd_vel_smoothed -> cmd_vel_safe` 三段均有连续非零样本。
- Web 安全门保持 `enabled=false`、`output=[0,0,0]`，未向实机执行运动。

完整 Smac `NavigateToPose` 实测仍要求先在网页完成当前开机的地图位姿标定。
