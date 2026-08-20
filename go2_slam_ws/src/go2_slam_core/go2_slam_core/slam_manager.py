#!/usr/bin/env python3
"""slam_manager 节点

职责：
1. 通过 unitree_api RPC 控制宇树内置建图（开启/停止并保存/停止节点）
2. 解析 /slam_info 状态（位姿等），发布 /slam/pose 与 /slam/status
3. 地图源自动选择并转发：/global_map(内置) > /map(外部SLAM) > /fallback/map(备用Python SLAM)
   -> 统一发布到 /slam/map，供 Web 界面使用
4. 服务：/slam/start_mapping、/slam/stop_mapping_and_save、/slam/save_map（均为 std_srvs/Trigger）
   停止/保存时地图名自动生成 map_YYYYmmdd_HHMMSS，PCD 存到 maps 目录；
   若备用 Python SLAM 有地图，同时保存为 nav2 格式 PGM+YAML。
"""

import json
import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry

from .slam_rpc import (
    SlamRpcClient,
    API_START_MAPPING,
    API_END_MAPPING,
    API_STOP_NODE,
)
from .fallback_slam import pointcloud2_to_xy

DEFAULT_MAPS_DIR = '/home/unitree/go2_slam_ws/maps'

# 内置 LIO 地图点云 -> 2D 栅格投影参数
LIO_RES = 0.05
LIO_GRID = 320          # 16m × 16m
# 扫描匹配锚定（抑制 LIO 里程计累积漂移）
MATCH_ON_OCC = 400      # 栅格占用格数超过此值才启用匹配修正
SCORE_MIN_PROJ = 0.10   # 匹配得分低于此值视为失败，回退 LIO 位姿
MATCH_MAX_D = 1.0       # 修正量与 LIO 位姿最大位移差(m)，超过视为匹配跳变
MATCH_MAX_YAW = 0.6     # 修正量与 LIO 位姿最大偏航差(rad)（转身时 LIO 旋转误差可达 15°+）
MATCH_INTERVAL = 0.3    # 匹配周期(s)
# 地图源优先级（高 -> 低）
MAP_SOURCE_PRIORITY = ['global_map', 'lio/map', 'map', 'fallback/map']


class SlamManager(Node):

    def __init__(self):
        super().__init__('slam_manager')

        # 单实例保护（flock 文件锁）：双 manager 会同时发 1801/同时发布地图，
        # 导致 LIO 状态机错乱（odom 停发）与前端数据串扰。已有实例运行则退出。
        import fcntl
        self._lock_f = open('/tmp/slam_manager.lock', 'w')
        try:
            fcntl.flock(self._lock_f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            print('已有一个 slam_manager 在运行，本实例退出（避免双实例冲突）',
                  file=sys.stderr)
            sys.exit(0)

        self.declare_parameter('maps_dir', DEFAULT_MAPS_DIR)
        self.maps_dir = self.get_parameter('maps_dir').value
        os.makedirs(self.maps_dir, exist_ok=True)

        self.rpc = SlamRpcClient(self)

        # 状态
        self.mapping = False          # 是否在建图
        self.engine_online = False    # 宇树内置 slam 服务是否在线
        self.last_info = 0.0          # 最近一次收到 /slam_info 的时间（存活探测用）
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stamp': 0.0}
        self.op_result = None         # (op, success, message) 最近一次后台操作结果

        # 地图源: name -> (msg, last_time)
        self.map_sources = {}
        self.create_subscription(OccupancyGrid, '/global_map', self._mk_map_cb('global_map'), 5)
        self.create_subscription(OccupancyGrid, '/map', self._mk_map_cb('map'), 5)
        self.create_subscription(OccupancyGrid, '/fallback/map', self._mk_map_cb('fallback/map'), 5)
        # 主地图源：原始点云 + LIO 位姿 -> 2D 栅格（实时投影，带射线自由区清洗）
        self.lio_grid = np.zeros((LIO_GRID, LIO_GRID), dtype=np.float32)  # -1=自由 0=未知 1=占用
        self._proj_t = 0.0
        # 扫描匹配锚定状态：把 LIO 位姿拉回已建栅格，抑制累积漂移
        self.match_on = False       # 地图足够丰富后启用
        self._match_t = 0.0         # 匹配限频
        self.match_delta = (0.0, 0.0, 0.0)  # 最近一次修正偏移 (dx, dy, dyaw)
        self.match_stats = {'tries': 0, 'ok': 0, 'score': 0.0}  # 匹配诊断统计
        self._last_match_score = 0.0
        self._match_fail = 0      # 连续匹配失败计数（清空修正偏移用）
        self.create_subscription(PointCloud2, '/unitree/slam_lidar/points',
                                 self.on_lidar_proj, 10)

        # 发布
        self.pub_map = self.create_publisher(OccupancyGrid, '/slam/map', 5)
        self.pub_pose = self.create_publisher(Odometry, '/slam/pose', 10)
        self.pub_status = self.create_publisher(String, '/slam/status', 10)
        self.pub_reset_fallback = self.create_publisher(Bool, '/fallback/reset_cmd', 10)

        # 宇树内置 slam 状态（位姿等）
        self.create_subscription(String, '/slam_info', self.on_slam_info, 10)
        # 内置 LIO 位姿（主位姿源，10Hz）
        self.lio_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stamp': 0.0}
        # 地图原点：首个 LIO 位姿。栅格以它为原点建图，位姿输出/匹配都用相对坐标，
        # 这样 manager 重启清栅格（LIO session 未重置、odom 已累积位移）时，
        # 地图与位姿仍然自洽，不会出现"定位错位"
        self.map_origin = None
        self.create_subscription(Odometry, '/unitree/slam_mapping/odom', self.on_lio_odom, 10)
        # 备用 SLAM 位姿（内置位姿过期时的回退源）
        self.fallback_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stamp': 0.0}
        self.create_subscription(Odometry, '/fallback/pose', self.on_fallback_pose, 10)

        # 周期任务
        self.create_timer(0.25, self.relay_map)     # 5Hz 转发地图
        self.create_timer(1.0, self.publish_status)  # 1Hz 状态
        self.create_timer(0.5, self.publish_pose)    # 2Hz 位姿（pos_info 无狗时可能不流，需定时发布）
        self.create_timer(3.0, self.check_engine)    # 3s 探测引擎在线（仅看 /slam_info 是否在流，不打扰建图）

        # 服务
        self.create_service(Trigger, '/slam/start_mapping', self.on_start_mapping)
        self.create_service(Trigger, '/slam/stop_mapping_and_save', self.on_stop_mapping_and_save)
        self.create_service(Trigger, '/slam/save_map', self.on_save_map)
        self.create_service(Trigger, '/slam/restart_mapping', self.on_restart_mapping)

        self.get_logger().info('slam_manager 启动, maps_dir=%s' % self.maps_dir)

    # ---------- 地图源 ----------

    def _mk_map_cb(self, name):
        def cb(msg):
            self.map_sources[name] = (msg, time.time())
        return cb

    def on_lidar_proj(self, msg):
        """主地图源：原始点云（雷达系）用 LIO 位姿变换到世界系，投影 2D 栅格。
        - z 带取雷达系 ±0.5m（主扫描带），与雷达画面一致，排除地面/高点
        - 射线自由区清洗：机器人->点的路径标记自由，位姿漂移时自动擦除旧残影
        - 扫描匹配锚定：地图足够丰富后，每帧先与已建栅格做 scan-to-map 匹配，
          把 LIO 位姿拉回墙面上，抑制里程计累积漂移；得分不足/跳变则回退 LIO 位姿
        - 只使用最新 LIO 位姿（过期则不更新，避免初始化期错误投影）"""
        now = time.time()
        # 投影限频 ~5Hz（地图更新无需 10Hz）
        if now - self._proj_t < 0.2:
            return
        self._proj_t = now
        if now - self.lio_pose['stamp'] > 1.0:
            return
        pts = pointcloud2_to_xy(msg, -0.5, 0.5, 800)
        if pts is None or len(pts) < 50:
            return
        x, y, yaw = self.lio_pose['x'], self.lio_pose['y'], self.lio_pose['yaw']
        # 转相对地图原点坐标（栅格中心 = 建图起始位姿）
        if self.map_origin is not None:
            ox0, oy0, oyaw0 = self.map_origin
            dxr, dyr = x - ox0, y - oy0
            c0, s0 = math.cos(-oyaw0), math.sin(-oyaw0)
            x = dxr * c0 - dyr * s0
            y = dxr * s0 + dyr * c0
            yaw = normalize_angle(yaw - oyaw0)
        # 扫描匹配锚定：占用内容足够后启用。
        # 每 MATCH_INTERVAL 做一次 scan-to-map 匹配得到修正偏移；中间帧直接用
        # "LIO 位姿 + 最近修正偏移"（每帧锚定、无滞后、不阻塞单线程）。
        if self.match_on:
            if now - self._match_t >= MATCH_INTERVAL:
                self._match_t = now
                self.match_stats['tries'] += 1
                m = self._match_scan(pts, x, y, yaw)
                if m is not None:
                    self.match_stats['ok'] += 1
                    self.match_stats['score'] = self._last_match_score
                    self._match_fail = 0
                    self.match_delta = (m[0] - x, m[1] - y,
                                        normalize_angle(m[2] - yaw))
                    x, y, yaw = m
                else:
                    # 连续失败说明当前环境匹配不可靠（空旷/退化），
                    # 挂着的旧修正量会引入偏置，累计 3 次后清空回退纯 LIO
                    self._match_fail += 1
                    if self._match_fail >= 3:
                        self.match_delta = (0.0, 0.0, 0.0)
            else:
                x += self.match_delta[0]
                y += self.match_delta[1]
                yaw += self.match_delta[2]
        else:
            occ_count = int((self.lio_grid > 0.55).sum())
            if occ_count > MATCH_ON_OCC:
                self.match_on = True
                self.get_logger().info(
                    '扫描匹配锚定启用（占用格 %d）' % occ_count)
        c, s = math.cos(yaw), math.sin(yaw)
        # LIO yaw 数值 = 雷达 x 真实方位 + 180°（实测箭头差 180° 反推），
        # 因此投影需用 -R(yaw)（补偿半圈），否则地图绕机器人转 180°（对称环境看不出，
        # 箭头方向会暴露）。
        wx = -pts[:, 0] * c + pts[:, 1] * s + x
        wy = -pts[:, 0] * s - pts[:, 1] * c + y
        ix = ((wx / LIO_RES) + LIO_GRID / 2).astype(np.int32)
        iy = ((wy / LIO_RES) + LIO_GRID / 2).astype(np.int32)
        rx = x / LIO_RES + LIO_GRID / 2
        ry = y / LIO_RES + LIO_GRID / 2
        # 射线自由区清洗（全向量化：一次生成所有插值点，替代逐点 linspace）
        n_steps = np.maximum(np.abs(ix - rx), np.abs(iy - ry)).astype(np.int32) + 1
        m = n_steps > 1
        if m.any():
            ns = n_steps[m]
            ixm, iym = ix[m], iy[m]
            reps = np.repeat(np.arange(len(ns)), ns)
            t = ((np.arange(int(ns.sum())) -
                  np.repeat(np.cumsum(ns) - ns, ns)) / np.repeat(ns, ns))
            xs = (rx + (ixm[reps] - rx) * t).astype(np.int32)
            ys = (ry + (iym[reps] - ry) * t).astype(np.int32)
            valid = (xs >= 0) & (xs < LIO_GRID) & (ys >= 0) & (ys < LIO_GRID)
            if valid.any():
                self.lio_grid[ys[valid], xs[valid]] = np.maximum(
                    -1.0, self.lio_grid[ys[valid], xs[valid]] - 0.4)
        # 端点占用
        v = (ix >= 0) & (ix < LIO_GRID) & (iy >= 0) & (iy < LIO_GRID)
        self.lio_grid[iy[v], ix[v]] = 1.0
        # 输出（int8[] 直接发 bytes，避免 10 万元素 tolist）
        data = np.full(LIO_GRID * LIO_GRID, -1, dtype=np.int8)
        occ = self.lio_grid > 0.55
        # 自由区为负值（射线清洗累积 -0.4/次，下限 -1）；0 = 未知
        free = self.lio_grid < -0.05
        data[occ.ravel()] = 100
        data[free.ravel()] = 0
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = LIO_RES
        m.info.width = LIO_GRID
        m.info.height = LIO_GRID
        m.info.origin.position.x = -LIO_GRID / 2 * LIO_RES
        m.info.origin.position.y = -LIO_GRID / 2 * LIO_RES
        try:
            m.data = data.ravel().tobytes()
        except Exception:
            m.data = data.tolist()
        self.map_sources['lio/map'] = (m, time.time())

    # ---------- 扫描匹配锚定（抑制 LIO 累积漂移） ----------

    def _match_scan(self, pts, gx, gy, gyaw):
        """scan-to-map 匹配：以 LIO 位姿为初值，在已建栅格中搜索最优对齐位姿。
        返回修正位姿 (x, y, yaw)；得分不足或跳变过大返回 None（回退 LIO 位姿）。
        所有 xy 候选批量评分（一次 gather），迭代数 ~23 次，耗时 ~2ms。"""
        best = (gx, gy, gyaw)
        best_s = -1.0
        # 评分用抽稀点（200 点，统计上足够）
        sample = pts[::4]
        # 粗搜索：yaw ±0.12(步0.03)，xy ±0.24(步0.12)
        ddxs = np.arange(-0.40, 0.401, 0.15)
        ddys = np.arange(-0.40, 0.401, 0.15)
        for dyaw in np.arange(-0.30, 0.301, 0.05):
            c, s = math.cos(gyaw + dyaw), math.sin(gyaw + dyaw)
            rot = sample @ np.array([[c, -s], [s, c]], dtype=np.float32)
            S = self._score_proj_batch(rot, ddxs, ddys, gx, gy)
            idx = np.unravel_index(int(np.argmax(S)), S.shape)
            sc = S[idx]
            if sc > best_s:
                best_s = sc
                best = (gx + ddxs[idx[0]], gy + ddys[idx[1]], gyaw + dyaw)
        if best_s < 0:
            return None
        # 精搜索两轮（每轮 7×7×7 候选，批量评分）
        for step, span in ((0.02, 0.06), (0.005, 0.015)):
            bx, by, byaw = best
            c, s = math.cos(byaw), math.sin(byaw)
            rot = sample @ np.array([[c, -s], [s, c]], dtype=np.float32)
            ddxs = np.arange(-span, span + step, step)
            ddys = np.arange(-span, span + step, step)
            for dyaw in np.arange(-span, span + step, step):
                c, s = math.cos(byaw + dyaw), math.sin(byaw + dyaw)
                rot = sample @ np.array([[c, -s], [s, c]], dtype=np.float32)
                S = self._score_proj_batch(rot, ddxs, ddys, bx, by)
                idx = np.unravel_index(int(np.argmax(S)), S.shape)
                sc = S[idx]
                if sc > best_s:
                    best_s = sc
                    best = (bx + ddxs[idx[0]], by + ddys[idx[1]], byaw + dyaw)
        self._last_match_score = float(best_s)
        if best_s < SCORE_MIN_PROJ:
            return None
        # 跳变保护：修正量与 LIO 位姿差异过大视为匹配失败
        if (math.hypot(best[0] - gx, best[1] - gy) > MATCH_MAX_D or
                abs(normalize_angle(best[2] - gyaw)) > MATCH_MAX_YAW):
            return None
        return best

    def _score_proj_batch(self, rot, ddxs, ddys, gx, gy):
        """对一批 xy 平移候选同时评分（全向量化，3D broadcasting）。
        rot: (N,2) 以候选 yaw 旋转后的点；ddxs/ddys: 平移增量 1D 数组。
        返回 (n_dx, n_dy) 分数矩阵。
        评分 = 占用命中比例 - 0.8×自由区命中比例（自由区为负值，与未知区分）。"""
        half = LIO_GRID / 2.0
        bx = (rot[:, 0] + gx) / LIO_RES + half            # (N,)
        by = (rot[:, 1] + gy) / LIO_RES + half            # (N,)
        ox = np.asarray(ddxs, dtype=np.float32) / LIO_RES  # (n_dx,)
        oy = np.asarray(ddys, dtype=np.float32) / LIO_RES  # (n_dy,)
        # (n_dx, 1, N) / (1, n_dy, N)
        ix = np.clip((ox[:, None, None] + bx[None, None, :]).astype(np.int32),
                     0, LIO_GRID - 1)
        iy = np.clip((oy[None, :, None] + by[None, None, :]).astype(np.int32),
                     0, LIO_GRID - 1)
        g = self.lio_grid[iy, ix]                        # (n_dx, n_dy, N)
        occ = (g > 0.4).mean(axis=2)
        free = (g < -0.15).mean(axis=2)
        return occ - 0.8 * free

    def relay_map(self):
        """按优先级转发存活的地图源到 /slam/map。"""
        now = time.time()
        for name in MAP_SOURCE_PRIORITY:
            entry = self.map_sources.get(name)
            if entry is not None and now - entry[1] < 3.0:
                msg = entry[0]
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                self.pub_map.publish(msg)
                return

    def current_map(self):
        now = time.time()
        entries = []
        for name, (m, t) in self.map_sources.items():
            entries.append((now - t, name, m))
        entries.sort()
        for age, name, msg in entries:
            if age < 3.0:
                return name, msg
        return None, None

    # ---------- 状态 ----------

    def on_lio_odom(self, msg):
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        if self.map_origin is None:
            self.map_origin = (msg.pose.pose.position.x,
                               msg.pose.pose.position.y, yaw)
            self.get_logger().info(
                '地图原点锚定: x=%.2f y=%.2f yaw=%.1f°' % (
                    self.map_origin[0], self.map_origin[1],
                    math.degrees(self.map_origin[2])))
        self.lio_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': 2.0 * math.atan2(qz, qw),
            'stamp': time.time(),
        }

    def on_fallback_pose(self, msg):
        yaw = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.fallback_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'stamp': time.time(),
        }

    def on_slam_info(self, msg):
        self.last_info = time.time()
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if d.get('type') == 'pos_info':
            p = d.get('data', {}).get('currentPose', {})
            self.pose = {
                'x': float(p.get('x', 0.0)),
                'y': float(p.get('y', 0.0)),
                'yaw': float(p.get('yaw', 0.0)),
                'stamp': time.time(),
            }

    def publish_pose(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        now = time.time()
        # 位姿优先级：内置 LIO(slam_mapping/odom) > slam_info pos_info > 备用 SLAM
        p = None
        if now - self.lio_pose['stamp'] < 2.0:
            p = self.lio_pose
        elif now - self.pose['stamp'] < 2.0:
            p = self.pose
        else:
            f = self.fallback_pose
            if now - f['stamp'] < 2.0:
                p = f
        if p is None:
            return
        if p is self.lio_pose and self.map_origin is not None:
            # LIO 位姿转相对地图原点（与栅格同系），并叠加最近扫描匹配修正——
            # 地图是按修正后位姿累积的，位姿输出必须一致，否则前端定位会漂
            ox0, oy0, oyaw0 = self.map_origin
            dxr, dyr = p['x'] - ox0, p['y'] - oy0
            c0, s0 = math.cos(-oyaw0), math.sin(-oyaw0)
            odom.pose.pose.position.x = dxr * c0 - dyr * s0
            odom.pose.pose.position.y = dxr * s0 + dyr * c0
            yaw = normalize_angle(p['yaw'] - oyaw0)
            if self.match_on:
                odom.pose.pose.position.x += self.match_delta[0]
                odom.pose.pose.position.y += self.match_delta[1]
                yaw = normalize_angle(yaw + self.match_delta[2])
        else:
            odom.pose.pose.position.x = p['x']
            odom.pose.pose.position.y = p['y']
            yaw = p['yaw']
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_pose.publish(odom)

    def check_engine(self):
        self.engine_online = (time.time() - self.last_info) < 5.0

    def publish_status(self):
        src_name, src_msg = self.current_map()
        s = {
            'engine': 'unitree' if self.engine_online else 'offline',
            'mapping': self.mapping,
            'pose': {k: round(v, 3) for k, v in self.pose.items()},
            'map_source': src_name,
            'map_size': (src_msg.info.width, src_msg.info.height, src_msg.info.resolution) if src_msg else None,
            'match': dict(self.match_stats),
            'maps_dir': self.maps_dir,
            'op_result': list(self.op_result) if self.op_result else None,
            'time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        }
        m = String()
        m.data = json.dumps(s)
        self.pub_status.publish(m)

    # ---------- 控制 ----------

    def on_start_mapping(self, req, resp):
        # 重置备用 SLAM 地图（后台操作里走服务，更可靠）
        self._spawn_op('start')
        resp.success = True
        resp.message = '已发送开启建图指令（后台执行中）'
        return resp

    def on_stop_mapping_and_save(self, req, resp):
        self._spawn_op('stop_save')
        resp.success = True
        resp.message = '已发送停止并保存指令（后台执行中）'
        return resp

    def on_save_map(self, req, resp):
        self._spawn_op('save')
        resp.success = True
        resp.message = '已发送保存指令（后台执行中）'
        return resp

    def on_restart_mapping(self, req, resp):
        """重新建图：清空当前地图并重新开始。"""
        self._spawn_op('restart')
        resp.success = True
        resp.message = '已发送重新建图指令（后台执行中）'
        return resp

    def _spawn_op(self, op):
        """后台线程执行宇树 RPC 操作（服务器响应可能延迟 8~30s），结果写入 op_result。"""
        def worker():
            try:
                if op == 'start':
                    self._op_start()
                elif op == 'stop_save':
                    self._op_stop_save()
                elif op == 'save':
                    self._op_save()
                elif op == 'restart':
                    self._op_restart()
            except Exception as e:
                self.op_result = (op, False, '操作异常: %s' % e)
                self.get_logger().error(str(e))
        threading.Thread(target=worker, daemon=True).start()

    def _op_start(self):
        self._clear_lio_map()
        self._reset_fallback_both()
        if self.engine_online:
            code, data = self.rpc.call(API_START_MAPPING, '{"data": {"slam_type": "indoor"}}')
            ok, info = self._parse(data, code)
            if ok:
                self.mapping = True
                self.op_result = ('start', True, '开始建图成功: %s' % info)
            else:
                self.op_result = ('start', False, '开始建图失败: %s' % info)
        else:
            self.op_result = ('start', True, '宇树内置建图未在线，已重置备用 SLAM 地图')
        self.get_logger().info('op_result: %s' % (self.op_result,))

    def _op_restart(self):
        """清空当前地图并重新开始建图。"""
        # 清空 LIO 投影地图 + 备用 SLAM 地图
        self._clear_lio_map()
        self._reset_fallback_both()
        if self.engine_online:
            code, data = self.rpc.call(API_START_MAPPING, '{"data": {"slam_type": "indoor"}}')
            ok, info = self._parse(data, code)
            if ok:
                self.mapping = True
                self.op_result = ('restart', True, '重新建图成功: %s' % info)
            else:
                self.mapping = False
                self.op_result = ('restart', False, '重新建图失败: %s' % info)
        else:
            self.mapping = True
            self.op_result = ('restart', True, '宇树内置建图未在线，已清空备用 SLAM 地图并重新开始')
        self.get_logger().info('op_result: %s' % (self.op_result,))

    def _op_stop_save(self):
        name = self._new_map_name()
        pcd_path = os.path.join(self.maps_dir, name + '.pcd')
        saved = []
        messages = []

        if self.engine_online:
            code, data = self.rpc.call(
                API_END_MAPPING, json.dumps({'data': {'address': pcd_path}}))
            ok, info = self._parse(data, code)
            if ok:
                saved.append(pcd_path)
                messages.append('内置建图已保存: %s' % info)
            else:
                messages.append('内置建图保存提示: %s' % info)
            # 停止建图节点（内置流程的一部分）
            self.rpc.call(API_STOP_NODE)

        # 备用 SLAM 地图保存为 nav2 格式 PGM+YAML
        fg = self._save_fallback_map(name)
        if fg:
            saved.extend(fg)
            messages.append('备用地图已保存')

        # 停止后清除当前地图：下次开启建图时从 0 开始
        self._reset_fallback_both()
        self.mapping = False
        ok = len(saved) > 0
        self.op_result = ('stop_save', ok, ' | '.join(messages) if messages else '未保存任何地图')
        self.get_logger().info('op_result: %s' % (self.op_result,))

    def _op_save(self):
        """不停止建图，仅保存当前地图快照。"""
        name = self._new_map_name()
        pcd_path = os.path.join(self.maps_dir, name + '.pcd')
        saved = []
        messages = []
        if self.engine_online:
            code, data = self.rpc.call(
                API_END_MAPPING, json.dumps({'data': {'address': pcd_path}}))
            ok, info = self._parse(data, code)
            if ok:
                saved.append(pcd_path)
                messages.append('内置建图已保存: %s' % info)
            else:
                messages.append('内置建图保存提示: %s' % info)
        fg = self._save_fallback_map(name)
        if fg:
            saved.extend(fg)
            messages.append('备用地图已保存')
        ok = len(saved) > 0
        self.op_result = ('save', ok, ' | '.join(messages) if messages else '未保存任何地图')
        self.get_logger().info('op_result: %s' % (self.op_result,))

    # ---------- 内部工具 ----------

    def _new_map_name(self):
        return 'map_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    def _parse(self, data, code):
        """解析 RPC 响应 JSON。返回 (成功?, 提示信息)。"""
        try:
            d = json.loads(data)
            if d.get('errorCode') == 0:
                return True, str(d.get('info', 'ok'))
            return False, str(d.get('info', 'error %s' % d.get('errorCode')))
        except Exception:
            if code == 0:
                return True, data
            return False, 'RPC 调用失败 code=%s' % code

    def _reset_fallback(self):
        pass

    def _clear_lio_map(self):
        """清空 LIO 实时投影栅格并复位位姿缓存（重新开启建图时旧地图必须清除）。"""
        self.lio_grid[:] = 0.0
        self.lio_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stamp': 0.0}
        self.match_on = False
        self._match_t = 0.0
        self.match_delta = (0.0, 0.0, 0.0)
        self.match_stats = {'tries': 0, 'ok': 0, 'score': 0.0}
        self._match_fail = 0
        self.map_origin = None   # 下个 LIO 位姿重新锚定地图原点
        self.get_logger().info('已清空 LIO 投影地图')

    def _reset_fallback_both(self):
        """复位备用 SLAM：话题 + 服务双通道（话题为主，单线程 fallback 下可靠）。"""
        msg = Bool()
        msg.data = True
        self.pub_reset_fallback.publish(msg)
        try:
            cli = self.create_client(Trigger, '/fallback/reset')
            if cli.wait_for_service(timeout_sec=2.0):
                future = cli.call_async(Trigger.Request())
                deadline = time.time() + 4.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.05)
            self.destroy_client(cli)
        except Exception as e:
            self.get_logger().warn('复位服务调用异常: %s' % e)

    def _save_fallback_map(self, name):
        """把备用 SLAM 的当前地图保存为 PGM+YAML（nav2 map_saver 格式）。"""
        _, msg = self.current_map()
        if msg is None:
            return None
        try:
            w, h = msg.info.width, msg.info.height
            res = msg.info.resolution
            data = msg.data
            pgm = bytearray()
            pgm_path = os.path.join(self.maps_dir, name + '.pgm')
            yaml_path = os.path.join(self.maps_dir, name + '.yaml')
            # 0=自由(254), 100=占用(0), -1=未知(205)
            for i in range(w * h):
                v = data[i]
                if v == -1:
                    pgm.append(205)
                elif v >= 65:
                    pgm.append(0)
                else:
                    pgm.append(254)
            header = b'P5\n%d %d\n255\n' % (w, h)
            with open(pgm_path, 'wb') as f:
                f.write(header + bytes(pgm))
            origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            yaml_text = (
                'image: %s\n'
                'resolution: %s\n'
                'origin: [%s, %s, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n'
            ) % (os.path.basename(pgm_path), res, origin[0], origin[1])
            with open(yaml_path, 'w') as f:
                f.write(yaml_text)
            return [pgm_path, yaml_path]
        except Exception as e:
            self.get_logger().error('保存备用地图失败: %s' % e)
            return None


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()
    # 单线程执行器：Foxy 的 MultiThreadedExecutor 会因高频重回调（点云投影）
    # 饿死其他订阅（如 /unitree/slam_mapping/odom），必须避免。
    # 服务回调不阻塞（RPC 在后台线程执行），单线程下一切正常。
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
