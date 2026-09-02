# GO2-W Nav2 集成

本工作空间在不更换宇树底层系统、LIO-SAM、SCAN-Planner 或控制协议的前提下，
增加一套 ROS 2 Humble Nav2 导航链。第一阶段默认运行在影子模式：Nav2 与 SCAN
接收同一个地图目标并并行计算，但底盘安全门只转发 SCAN。Nav2 不使用 TensorRT，
也不修改 GO2-W 固件。

## 运行架构

```text
Web 目标（nav_map）
  ├─ 原 Web A* 路径（odom） ──> SCAN 局部规划 ──> SCAN cmd
  └─ NavigateToPose ──> Smac State Lattice ──> MPPI ──> velocity smoother
                                                     └─ Collision Monitor
                                                          └─ Nav2 safe cmd

SCAN cmd ─┐
          ├─ chassis_safety_gate（只选一个后端）──> Unitree Sport API
Nav2 cmd ─┘
```

地图定位仍由现有 Web 自动定位负责。Web 内部历史变换满足
`p_odom = T_odom_map * p_map`；发布给 Nav2 的标准 TF 是其逆变换
`nav_map -> odom`。定位无效或心跳过期时不发布该 TF，因此 Nav2 全局规划会
自然停在未就绪状态，安全门也独立拒绝自主运动。

## 关键配置

| 环节 | 当前实现 |
|---|---|
| 全局规划 | Smac State Lattice，5 cm、0.5 m 最小转弯半径、允许倒车 |
| 局部控制 | MPPI DiffDrive，20 Hz，48×0.05 s 预测窗，1000 条采样 |
| 车体 | 0.62 m × 0.36 m 多边形 footprint，额外 2 cm padding |
| 速度 | 前进 0.50、倒车 0.20、角速度 0.45 rad/s |
| 动态障碍 | XT16 PointCloud2 同时进入局部/全局 costmap |
| 末端安全 | Collision Monitor：停车区、减速区、1.5 s 碰撞预测 |
| 恢复 | 清 costmap、等待 3 s、后退 0.20 m、旋转 45°，总次数有界 |

配置文件位于
`nav2_overlay/src/go2_nav2_bringup/config/nav2_go2w.yaml`，行为树位于
`nav2_overlay/src/go2_nav2_bringup/behavior_trees/go2w_navigate.xml`。

## 启动与验收

```bash
# 构建 Humble overlay；仅在 Dockerfile 变化时需要 --image
./build_nav2.sh
./build_nav2.sh --image

# 基础导航已经运行且底盘锁定时，单独启动影子链
./start_nav2_shadow.sh

# 停止影子链
./stop_nav2_shadow.sh
```

`start_navigation.sh` 已串入 Nav2 影子服务。重启后尚未自动定位时，局部安全
节点先进入 active，全局 planner/BT 等待 `nav_map -> odom`；自动定位成功后
生命周期管理器会继续完成激活。

检查：

```bash
docker exec go2_nav2_shadow bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/go2_slam_ws/nav2_overlay/install/setup.bash
  ros2 lifecycle get /planner_server
  ros2 lifecycle get /controller_server
  ros2 lifecycle get /collision_monitor
'

source ./setup_env.sh
ros2 topic echo /go2/nav2/shadow_metrics
curl -s http://127.0.0.1:8890/api/navigation/backend
```

## 切换真实控制后端

默认后端永远是 `scan`，安全门重启后也恢复为 `scan`。只有完成影子数据对比、
低速架空轮测试和封闭场地测试后，才切换 Nav2：

```bash
# 必须先取消导航并确认底盘 enabled=false
./select_navigation_backend.sh nav2

# 回退
./select_navigation_backend.sh scan
```

切换接口会清除 SCAN/Nav2 两边的旧目标，锁定底盘，再请求安全门更换输入。
安全门在已启用状态下拒绝切换，并要求切换后重新启用、重新发送目标。直接向
ROS 话题发布后端名称也不能绕过安全门的定位、LowState、倾角、心跳、里程计、
指令超时和速度/加速度限制。

## Corner case 处理

- 动态障碍短时出现：MPPI 绕行；Collision Monitor 必要时先减速/停车。
- 全局路径失效：BT 立即重规划；路径有效时最多每 5 秒刷新，避免无意义高频重算。
- 目标落在障碍附近：Web 先吸附到同一自由连通区，Smac 再做 footprint 碰撞检查。
- 狭窄通道/转弯不可行：State Lattice 按实际转弯半径搜索，不把中心点 A* 当成可执行路径。
- 控制失败或卡住：局部/全局 costmap 分层清理并执行次数、距离均有界的恢复动作。
- 点云、TF、里程计或命令超时：对应 Nav2 层停止输出，底盘安全门再独立锁停。
- 后端进程退出：未选中的影子链不影响底盘；选中链命令超过 0.30 s 会触发安全停车。
