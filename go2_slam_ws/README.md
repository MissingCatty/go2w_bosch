# GO2-W LIO-SAM 建图工作空间

GO2-W（轮式）+ Pandar XT16 + 机身内置 IMU 的 ROS2 建图链路，提供浏览器实时地图、点云、位姿和三维地图保存。

仓库已加入端侧语义记忆适配：复用前置相机和已对齐的 map 位姿，以轻量
SQLite/WAL 保存描述和时空坐标，检索结果只生成需要人工确认的安全候选点。
VLM 已适配本机 Qwen3.5-2B Q8_0 + Q8 视觉投影器 + llama.cpp CUDA 服务，自动采集
默认仍关闭；硬件部署、配置和安全边界见
[`REMEMBR_EDGE_INTEGRATION.md`](REMEMBR_EDGE_INTEGRATION.md)。

默认导航使用 Web A* 全局路径、SCAN-Planner 局部 B 样条以及项目自研的闭环
控制和实时脱困节点。实验性的 Nav2 影子链保留在源码中供离线对比，但不会由
`start_navigation.sh` 构建或启动。

## 当前数据链路

```text
Pandar XT16
  └─ xt16_driver ── /unitree/slam_lidar/points (10 Hz, time 字段为 ns)
       └─ C++ 抽稀 ── /go2_slam/lidar_preview (Web 预览专用)

GO2-W 机身内置 IMU
  └─ /lowstate (500 Hz)
       └─ imu_attitude_bridge
            ├─ 启动时静止标定 2 秒（加速度和陀螺零偏）
            └─ /dog_imu_lio (250 Hz, sensor_msgs/Imu)

点云 + /dog_imu_lio
  └─ Humble Docker: LIO-SAM
       ├─ /lio_sam/mapping/odometry
       └─ /lio_sam/mapping/map_global
            └─ Foxy web_server (:8890)
                 ├─ 2D/3D 地图、实时雷达、位姿与健康状态
                 └─ maps/map_*.npz（三维点云）
```

注意：机器人还有 `/utlidar/imu`，它来自摄像头/副传感器，不属于这条链路。当前建图只使用 `/lowstate.imu_state` 的机身内置 IMU。

## 启动和停止

```bash
cd ~/go2_slam_ws

# 启动前让机器人静止约 3 秒，供内置 IMU 自动标定
./start.sh

# 源码或参数改动后强制重建 host 和 Humble 工作空间
./start.sh --build

# 重启后一键拉起完整导航链路（含 SCAN，音量设为 10%，底盘保持锁定）
./start_navigation.sh

# 同时重编主机、LIO-SAM、SCAN 和 VLM 组件
./start_navigation.sh --build

# 可选实验：单独管理 Nav2 影子链（默认不启动、不接管底盘）
./build_nav2.sh
./start_nav2_shadow.sh
./stop_nav2_shadow.sh

# 单独构建、启动或停止端侧 Qwen3.5-2B INT8 VLM
./build_remembr_vlm.sh
./start_remembr_vlm.sh
./stop_remembr_vlm.sh

# 交互式存入 DeepSeek Flash reasoner 密钥（不要把密钥发到聊天中）
./configure_remembr_api.sh

# 停止驱动、桥接、LIO-SAM 和 Web；不会删除 maps/
./stop.sh
```

脚本使用用户级 systemd 临时服务监管进程，SSH/终端退出后不会带走建图链。重复执行 `start.sh` 会复用健康实例；主机桥接重启时会同步重置 LIO-SAM，避免旧状态污染新地图。

浏览器地址由启动脚本打印，通常为：

```text
http://192.168.123.18:8890
```

## 地图保存

网页右下方点击“保存地图”，保存当前回环修正后的全局三维点云：

```text
maps/map_YYYYmmdd_HHMMSS_mmm.npz
```

NPZ 内容：

- `map`：`N x 3` float32，全局地图 xyz；
- `ground`：`M x 3` float32，地面高度带 xyz。

地图或位姿离线时保存会被拒绝，防止把过期快照当成有效地图。

## 导航地图派生

原始 NPZ 保持不动，使用离线工具生成校平、去离群、占据证据过滤、弱小
噪点过滤和按高度截取后的导航地图：

```bash
~/go2_slam_ws/tools/prepare_navigation_map.py \
  ~/go2_slam_ws/maps/map_20260811_155640_273.npz \
  --clear-start-x -0.05 --clear-start-y 0.21 --clear-start-radius 0.80
```

输出位于 `maps/navigation/`：

- `*_nav.pgm/yaml`：5 cm 原始占用图；
- `*_nav_inflated.pgm/yaml`：按 23 cm 半径膨胀的 Go2-W 规划图；
- `*_nav_obstacles.pcd`：保留用于地图检查/兼容的 2.5D 静态障碍层；当前
  SCAN 局部栅格只使用实时雷达，不注入该静态点云；
- `*_nav.json`：地面平面、点数、边界和坐标对齐参数。

`clear-start-*` 只用于清理由实时点云确认不存在实体障碍的机器人初始
占位区域；它会在安全膨胀之后再次生效，避免边界噪声把起点围成孤岛。
换地图或换起点时必须重新测量，不能照搬。转换结果的 JSON 会记录原始和
膨胀栅格的自由空间连通性，`start_in_largest_component` 应为 `true`。
默认弱小噪点过滤只删除膨胀前面积不超过 0.075 m²、且少于 3 个非地面
回波支撑的孤立障碍；墙体和有重复非地面回波的小物体会保留。Web A* 会把
目标吸附到起点所在连通区，避免先吸附到附近自由孤岛后误报无路。
当前静态图到重启后
导航页提供“重启后自动定位”：只需在静态图上粗选机器人当前
X/Y 区域，朝向角可选（0° 沿 +X，90° 沿 +Y）。系统采集多帧
`/lio_sam/mapping/cloud_registered` 点云，先做粗分辨率距离场搜索，再用
保留高度特征的 SE(2) ICP 精配准。只有重合率、RMSE、匹配点数、粗选修正量和
解唯一性都达标才写入 `map -> odom` 变换，然后统一转换全局/局部路径和
Web 位姿；不达标时底盘继续锁定。
标定会绑定本次 Linux boot ID 和当前导航地图内容；重启或切换/重建
导航基准后自动失效。未标定时，Web 目标接口、导航底盘启用接口和底盘
安全门三层均拒绝自主导航运动，离线假起点路径预演仍可使用。

导航页的“取消导航”会同时清除 Web 目标、全局/局部路径，并通过
`/scan_planner/cancel` 让 SCAN 状态机回到等待目标、控制器持续发布零速度；
规划器保持运行，之后可以直接重新选择目标点。

导航页还提供默认关闭的“键盘控制”开关。开启时会先取消路径导航，再通过
同一底盘安全门显式启用互斥的浏览器遥控输入：`W/S` 前进/后退，`A/D`
左/右侧移，`Q/E` 左/右转向。前进、侧移和转向上限分别为 0.40 m/s、
0.10 m/s 和 0.30 rad/s。松开按键立即发送零速度；遥控指令超过 0.35 秒、
持有控制权的 WebSocket 断开、页面失焦/切到后台、切换页面或关闭开关时，
均会自动停车、清除遥控所有权并锁定底盘。另一个网页不能同时取得控制权。
键盘控制使用机身坐标系速度，不依赖地图、LIO 里程计或 SCAN；它只要求
Sport API、LowState、Web 心跳和倾角保护健康，因此未完成地图定位时也可用。

导航页还提供“离线路径预演”：可填写或在 3D 地图上选择假起点，再沿用
目标坐标运行静态 5 cm 安全栅格 A*。预演只返回并显示路径，不依赖实时狗
位姿，不启动 SCAN/摄像头，也不发布 ROS 全局路径；只有“规划并发送”才会
按需启动实时导航链路。

地图管理页提供非破坏性的“手绘擦除”：在 2D 俯视预览中按住鼠标沿轮廓
绘制，松开后自动闭合成不规则多边形；可连续添加多个区域、撤销最后一个或
全部清空，最终统一另存为一个新 NPZ，只删除所有多边形内相对拟合地面
0.08–2.20 m 的 `map` 点；保存的 `ground`
数组、低于 0.08 m 的地面点和高于 2.20 m 的顶棚/楼顶点均强制保留。
编辑当前导航基准图时会自动重建规范名称的导航栅格和障碍 PCD，原始 NPZ
与旧导航派生文件均保留备份。一次最多 50 个区域，不限制总面积；单个区域
最多 500 个轮廓点，全部区域合计最多 5000 个轮廓点。

地图列表中的“设为基准”可把任意已校验 NPZ 切换为当前导航基准。该操作只
允许在地图管理页且 SCAN-Planner 已停止时执行，会在隔离目录生成导航栅格、
安全膨胀图和障碍 PCD，验证完成后原子切换并热加载；失败时自动恢复旧基准。

默认占据证据过滤会保留满足任一条件的障碍连通片：至少 8 个 5 cm 栅格、
至少 12 个源点，或至少 0.70 m 垂直跨度。这样可保留墙体、重复观测的小物体
和细杆，同时避免单帧散点经 23 cm 安全膨胀后堵塞大面积通道。Web 全局规划
直接使用已经膨胀的 5 cm 栅格，不再二次缩成 10 cm。

## 关键修正与参数

- XT16 点云 `time` 是纳秒，LIO-SAM 内部先乘 `1e-9` 转成秒再去畸变；
- 出厂外参：平移 `(0.1701, 0, 0.0908)`，雷达相对机身 yaw `+90°`；
- 姿态外参使用 `+90°`，机身 IMU 向量转雷达系使用逆矩阵 `-90°`；
- 实测重力模长 `9.46036 m/s²`；
- GO2-W 室内低速建图只用 IMU 旋转作为 scan-to-map 初值，平移由激光匹配求解，避免静止时预积分平移把配准推入错误局部解；
- z、roll、pitch 有平面运动约束，全局图 1 Hz 发布。

主要配置和实现：

```text
~/dddmr_navigation/src/lio_sam/config/params.yaml
~/dddmr_navigation/src/lio_sam/src/imageProjection.cpp
~/dddmr_navigation/src/lio_sam/src/mapOptmization.cpp
~/go2_slam_ws/src/go2_imu_bridge/src/imu_attitude_bridge.cpp
~/go2_slam_ws/src/go2_slam_web/go2_slam_web/web_server.py
```

## 状态检查

```bash
# Web 汇总状态；healthy 应为 true
curl -s http://127.0.0.1:8890/api/status

# 用户级服务
systemctl --user status go2-xt16 go2-slam-host go2-lio-sam

# 关键发布端
source ~/go2_slam_ws/setup_env.sh
ros2 topic info /unitree/slam_lidar/points
ros2 topic info /dog_imu_lio
ros2 topic info /lio_sam/mapping/odometry
ros2 topic info /lio_sam/mapping/map_global

# 运行日志
tail -f /tmp/xt16_driver_$(id -u).log
tail -f /tmp/go2_slam_launch_$(id -u).log
tail -f /tmp/dddmr_lio_$(id -u).log
docker logs -f dddmr_humble
```

正常频率约为：XT16 `10 Hz`、机身 LowState `500 Hz`、LIO IMU 桥 `250 Hz`、LIO 地图位姿 `约 5 Hz`、全局地图 `1 Hz`。

## Web API

| 接口 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 建图控制台 |
| `/api/status` | GET | 位姿、频率、地图统计和健康状态 |
| `/api/navigation/backend` | GET/POST | 查看或在底盘锁定时切换 SCAN/Nav2 后端 |
| `/api/save` | POST | 保存当前三维全局地图 |
| `/api/maps` | GET | 已保存地图列表 |
| `/api/download?name=...` | GET | 下载地图 |
| `/ws` | WebSocket | 5 Hz 实时推流；地图数据 1 Hz 更新 |

## 目录说明

```text
go2_slam_ws/
├── start.sh / stop.sh       # 全链路生命周期
├── setup_env.sh             # 清理 ROS1 环境并加载 Foxy + CycloneDDS
├── module/unitree_slam/     # XT16 驱动及原厂模块副本
├── maps/                    # Web 保存的三维地图
└── src/
    ├── go2_imu_bridge/      # C++ 内置 IMU 标定/桥接 + 雷达预览抽稀
    ├── go2_slam_core/       # 旧 Python bridge/fallback 源码保留但不启动
    ├── go2_slam_web/        # Tornado Web/WS 与前端
    └── go2_slam_bringup/    # host launch
```

本机同时安装 ROS1 Noetic、ROS2 Foxy，手工运行 ROS 命令前必须先 `source ~/go2_slam_ws/setup_env.sh`，否则可能加载到错误的 Python 消息包。LIO-SAM 运行在 ROS2 Humble 容器中，通过 host 网络和 CycloneDDS 与 Foxy 互通。
