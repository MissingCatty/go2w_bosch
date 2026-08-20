# GO2-W 开机启动全流程(建图 / 导航)

> 机器狗整机重启后的标准启动步骤。按顺序执行,每步有验证命令。
> 已验证于 2026-08-08(重启后恢复建图链路)。
> 2026-08-10 更新: xt16_driver 崩溃已解决(见踩坑速查),启动顺序建议先起容器再起驱动。
> sudo 密码: `123`

---

## 1. 恢复 SLAM 栈(host,必做)

```bash
echo 123 | sudo -S bash /home/unitree/go2_slam_ws/start.sh
```

**注意**: 脚本最后 `ros2 launch` 是前台进程,脚本不会自行退出——属正常,不用等它。
启动包含: USB3 供电激活 → xt16_driver(雷达驱动) + unitree_slam(建图服务) + web_server(浏览器控制台 http://<机器人IP>:8890)。

**验证(10 秒后)**:

```bash
pgrep -af "xt16_driver|unitree_slam|web_server" | grep -v grep
# 应看到 3 个进程: ./xt16_driver  ./unitree_slam  ...web_server
```

**日志**: `/tmp/xt16_driver.log`(雷达),`/tmp/unitree_slam.log`(SLAM)。
雷达刚启动时日志里出现 `Abnormal time information, this frame data is discarded` 是正常丢帧,几秒后消失。

> ⚠️ 不要用 host 的 `ros2 topic hz` 验证——Foxy 工具链报
> `ValueError: Expected the full name of a message`。验证一律在容器里做。

---

## 2. 启动建图模式(lego_loam)

```bash
cd /home/unitree/dddmr_navigation
nohup bash run_xt16_mapping.sh run > /tmp/mapping_console.log 2>&1 &
```

**验证(25 秒后,容器内)**:

```bash
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /unitree/slam_lidar/points --window 5'
# 应输出 ~10Hz
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /key_poses --window 5'
# 应输出 ~10Hz
```

**PC rviz 看效果**(Fixed Frame = `map`):

| 显示 | 话题 |
|---|---|
| 原始点云 | `/unitree/slam_lidar/points` |
| 关键帧轨迹 | `/cloud_keypose_6d` 或 `/key_poses` |
| 面/角特征 | `/recent_surf_cloud` / `/recent_corner_cloud` |
| 位姿图 | `/pose_graph` |

PC 与机器人需同网段(192.168.123.x)才能收到话题。

**关键帧规则**: 位移 >1m 或转角 >1° 才加关键帧——机器人静止时地图不增长是**正常现象**。

---

## 3. 启动导航模式(mcl_3dl 定位 + P2P 导航)

前置: 第 1 步 SLAM 栈必须已跑。

```bash
cd /home/unitree/dddmr_navigation
nohup bash run_xt16_mapping.sh navigation > /tmp/nav_console.log 2>&1 &
```

启动内容: odom_bridge + cmd_vel_sport_bridge(ROS2 cmd_vel → sport API)+ xt16_navigation.launch(20 节点)。

**验证(30 秒后)**:

```bash
# 节点数(容器内)
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && ros2 node list | wc -l'   # ~20+
# 定位收敛
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /mcl_3dl/estimated_pose --once'
# cmd_vel 桥活着
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /cmd_vel --window 5'  # 有移动指令时 ~20Hz
```

**运动测试**: `docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && source /root/dddmr_navigation/install/setup.bash && python3 /root/dddmr_navigation/cmd_vel_test.py'`
(0.1 m/s 微速前进 1.5s,注意手放遥控器)

**发导航目标**: 
```bash
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && source /root/dddmr_navigation/install/setup.bash && python3 /root/dddmr_navigation/send_goal_action.py X Y YAW [z]'
```

**紧急停止**: 
```bash
# 1. 杀导航节点(停止发指令)
sudo -S kill <p2p_move_base_node pid> <clicked2goal pid> <<< "123"
# 2. 发 3s 零速度(容器内)
docker exec dddmr_humble bash -c 'source /opt/ros/humble/setup.bash && python3 /root/dddmr_navigation/stop_robot.py'
```

---

## 4. 常用命令速查

```bash
# 进容器
docker exec -it dddmr_humble bash
# 重建(改 C++ 代码后)
bash /home/unitree/dddmr_navigation/run_xt16_mapping.sh build
# 保存地图(建图模式下,容器内)
python3 /tmp/save_pcd.py    # 输出到 /root/dddmr_maps/<时间戳>/ 再 docker cp 出来
# 紧急停止脚本(发 3s 零速度)
python3 /root/dddmr_navigation/stop_robot.py
# 停容器
docker stop dddmr_humble
```

---

## 5. 踩坑速查(勿重复踩)

| 问题 | 处理 |
|---|---|
| host `ros2 topic hz` 报 ValueError | 用容器内验证(见上) |
| 容器起不来 "No such container: dddmr_humble" | 旧容器没停:`docker stop dddmr_humble` 后重跑 |
| `bash: run_xt16_mapping.sh: No such file` | 先 `cd /home/unitree/dddmr_navigation` |
| `/lego_loam_map` 全 NaN | 可视化节点 bug,别加进 rviz,用 `cloud_keypose_6d` |
| `/laser_cloud_surround` `/aft_mapped_to_init` 无数据 | 死代码话题,从不发布 |
| lego_loam 报 footprint→sensor TF 错误 | 无 footprint 帧,忽略(不影响建图) |
| rclpy `rate.sleep()` 挂起 | 容器环境 bug,脚本统一用 `time.sleep` |
| sport 桥 `'NoneType' ._ref` | 缺 `ChannelFactoryInitialize(0, "eth0")`(桥脚本已内置) |
| A* "No path found" | 已修:go2_xt16_nav_config.yaml 中 `enable_edge_detection: false` + `plugins: ["map"]`(勿改回) |
| 关键帧不增长 | 正常,需移动机器人(>1m 或 >1°) |
| xt16_driver 启动即崩(日志尾 `dq.builtin: type [INVALID...]`) | CycloneDDS 0.10.2 类型 discovery 崩溃,已用补丁库根治(2026-08-10 稳定)。补丁说明与字节见 `/home/unitree/go2_slam_ws/LIBDDSC_PATCH.md`;回滚原版: `echo 123 \| sudo -S cp /usr/local/lib/libddsc.so.bak.20260810 /usr/local/lib/libddsc.so`(注意 module/lib 需同步)。若重启后崩溃:先确认两处库 md5 一致 = b8fb6016,再重启 start.sh |
| 驱动日志里话题名带 `rt/` 前缀 | 正常,CycloneDDS 显示格式,实际话题是 `/unitree/slam_lidar/points` |
| 只重启 web_server(改 web_server.py / static 后) | 杀旧进程后手动起: `echo 123 \| sudo -S bash -c 'source /home/unitree/go2_slam_ws/setup_env.sh > /dev/null 2>&1; nohup /usr/bin/python3 /home/unitree/go2_slam_ws/install/go2_slam_web/lib/go2_slam_web/web_server --ros-args -r __node:=web_server --params-file /tmp/launch_params_s1o_xk_a > /tmp/web2.log 2>&1 &'`。**必须**: source setup_env.sh(否则缺 CYCLONEDDS_URI → 狂刷 `std::bad_alloc` 崩溃);root 启动(kill 旧进程要 sudo);参数文件是 root 600,unitree 读不了。web_server.py 走 egg-link→src,改 src 即生效,无需 colcon build |
