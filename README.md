# GO2-W Bosch Mapping and Navigation

GO2-W 室内三维建图、地图管理和自主导航工程。系统使用机身内置 IMU、
Pandar XT16、LIO-SAM、Web 静态栅格 A* 和经过实机适配的 SCAN-Planner，
并通过默认锁定的底盘安全门接入 Unitree Sport API。当前版本还包含端侧
Remembr 语义记忆链（Qwen3.5-2B INT8 VLM + DeepSeek Flash reasoner）。默认
导航不启动 Nav2，使用 Web A* + SCAN 自研闭环控制和脱困链路。

## 目录

```text
go2_slam_ws/                 Foxy 主机端 IMU、Web、地图管理和底盘安全门
scan_planner_ws/             Humble SCAN-Planner 与 GO2-W 适配层
dddmr_navigation/src/lio_sam Humble LIO-SAM 实机修改版
config/cyclonedds.xml        Foxy/Humble 跨环境 DDS 配置
```

`scan_planner_ws/src/SCAN-Planner` 基于
[wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的
`ros2-community` 分支，保留其 Apache-2.0 `LICENSE` 和 `NOTICE`。本项目在其上
增加 GO2-W 碰撞体、实时障碍短期记忆、静态墙体边界、全局参考保持、局部等待
重试、闭环跟踪看门狗和实机安全接口。

## 数据链路

```text
XT16 + GO2-W 内置 IMU
        -> LIO-SAM 位姿/去畸变点云
        -> Web 静态地图 A* 全局路径
        -> SCAN 实时三维局部避障与 B 样条
        -> 闭环控制器
        -> 默认锁定的底盘安全门
        -> Unitree Sport API
```

机器人另有摄像头 IMU，但当前 LIO 链路只使用 `/lowstate` 中的机身内置 IMU。

## 部署前提

仓库只保存源码和可复现配置，不包含以下机器相关或大体积数据：

- `go2_slam_ws/module/unitree_slam` 原厂 XT16 驱动和动态库；
- ROS/colcon 的 `build`、`install`、`log`；
- Docker 镜像 `dddmr_humble:nav`；
- 实测 NPZ/PCD/PGM 地图及地图配准结果；
- SSH 密钥、运行日志和录包数据。

部署时目录应位于 `/home/unitree`：

```text
/home/unitree/go2_slam_ws
/home/unitree/scan_planner_ws
/home/unitree/dddmr_navigation
/home/unitree/cyclonedds_ws/cyclonedds.xml
```

将本仓库相应子目录同步到上述位置，并单独安装原厂模块、ROS 2 Foxy/Humble
环境、CycloneDDS、Unitree SDK 和所需 Docker 镜像。

## 构建与启动

```bash
cd /home/unitree/go2_slam_ws
./start_navigation.sh --build
```

普通重启后：

```bash
cd /home/unitree/go2_slam_ws
./start_navigation.sh
```

启动脚本不会自动启用真实底盘。进入 Web 导航页完成当前开机的地图自动定位，
确认环境后再由操作者手动启用底盘。

更完整的建图、地图后处理、Web API 和导航说明见：

- `go2_slam_ws/README.md`
- `go2_slam_ws/REMEMBR_EDGE_INTEGRATION.md`
- `go2_slam_ws/NAV2_GO2W_INTEGRATION.md`
- `scan_planner_ws/README_GO2W.md`
- `go2_slam_ws/NAVIGATION_IMPORTANT_BUGS.md`

## 安全说明

不要绕过 `go2_chassis_safety_gate` 将 `/scan_planner/cmd_vel_test` 直接连接到底盘。
安全门负责显式上锁、心跳、定位有效性、姿态、点云、里程计和命令超时检查。
任何传感器、坐标系、机器人尺寸或速度参数改动，都应先在底盘锁定状态验证。
