#!/usr/bin/env python3
"""fallback_slam 节点 — 备用 2D SLAM（纯 Python + numpy，零外部依赖）

用途：当宇树内置建图的地图输出暂不可用时（无狗控制数据 / 静态调试），
本节点直接用雷达点云做“栅格相关匹配”的 2D SLAM，保证建图画面实时可见：

  输入: /unitree/slam_lidar/points  (sensor_msgs/PointCloud2)
  输出: /fallback/map   (nav_msgs/OccupancyGrid)  分辨率 0.05m，16m×16m
        /fallback/pose  (nav_msgs/Odometry)
  复位: /fallback/reset_cmd (std_msgs/Bool) — True 时清空地图并回到原点

注意：本节点仅作兜底；实际建图以宇树内置 SLAM 为准。
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid, Odometry

RES = 0.05        # 栅格分辨率 (m)
GRID = 320        # 栅格边长 -> 16m × 16m
HALF = GRID / 2.0
Z_MIN, Z_MAX = -0.8, 0.8      # 投影到 2D 的 z 范围（相对雷达安装面）
MAX_SCAN_PTS = 1000           # 匹配用点云上限（均匀抽稀）
LOG_HIT, LOG_FREE = 0.5, -0.4   # 占用/自由累积量（自由区为负值，可与未知区分）
SCORE_MIN = 0.12              # 匹配分数低于此值视为失败，跳过该帧
ADD_MOVE, ADD_YAW = 0.10, 2.0  # 加入地图的最小位移(m)/转角(°)


class FallbackSlam(Node):

    def __init__(self):
        super().__init__('fallback_slam')
        # 雷达安装校准：雷达前向相对狗前向的偏航角（度，正=逆时针/左）。
        # GO2-W 头盔 XT16 侧装，出厂标定 T_Dog2lidar yaw=1.57(90°)
        self.declare_parameter('lidar_yaw_offset', -90.0)
        self._yaw_off = math.radians(self.get_parameter('lidar_yaw_offset').value)
        self.grid = np.zeros((GRID, GRID), dtype=np.float32)  # 0=未知, 1=占用
        self.lock = threading.Lock()
        self.pose = np.array([0.0, 0.0, 0.0])   # x, y, yaw
        self.pose_prev = None
        self.has_map = False
        self.scan = None        # (N,2) 狗坐标系点
        self.last_add = 0.0
        self._last_match_t = 0.0
        self._t_log = 0.0

        self.create_subscription(PointCloud2, '/unitree/slam_lidar/points',
                                 self.on_lidar, 10)
        # 复位指令优先走服务（话题 /fallback/reset_cmd 保留兼容）
        self.create_service(Trigger, '/fallback/reset', self.on_reset_srv)
        self.pub_map = self.create_publisher(OccupancyGrid, '/fallback/map', 5)
        self.pub_pose = self.create_publisher(Odometry, '/fallback/pose', 10)
        self.create_timer(0.25, self.publish_map)   # 4Hz

        self.get_logger().info('fallback_slam 启动 (栅格 %dx%d @ %.2fm)' % (GRID, GRID, RES))

    # ---------- 输入 ----------

    def on_lidar(self, msg):
        now = time.time()
        # 匹配频率上限 ~6Hz：点云 10Hz 输入，避免重匹配阻塞单线程执行器
        if now - self._last_match_t < 0.16:
            return
        pts = pointcloud2_to_xy(msg, Z_MIN, Z_MAX, MAX_SCAN_PTS)
        if pts is None or len(pts) < 50:
            return
        # 雷达系 -> 狗系（旋转 lidar_yaw_offset），保证地图/位姿在狗坐标系下
        if self._yaw_off:
            c, s = math.cos(self._yaw_off), math.sin(self._yaw_off)
            pts = pts @ np.array([[c, -s], [s, c]], dtype=np.float32)
        self._last_match_t = now
        with self.lock:
            self.scan = pts
            self._process(pts)

    def on_reset_cmd(self, msg):
        if msg.data:
            self.reset_map()

    def on_reset_srv(self, req, resp):
        self.reset_map()
        resp.success = True
        resp.message = '备用 SLAM 已复位'
        return resp

    def reset_map(self):
        with self.lock:
            self.grid[:] = 0.0
            self.pose[:] = 0.0
            self.pose_prev = None
            self.has_map = False
            self.last_add = 0.0
            self.scan = None
        self.get_logger().info('备用 SLAM 已复位')

    # ---------- 核心: 匹配 + 建图 ----------

    def _process(self, pts):
        t0 = time.time()
        # 第一帧：直接初始化地图
        if not self.has_map:
            self._add_scan(pts, self.pose)
            self.has_map = True
            self.last_add = t0
            return
        # 扫描匹配（粗到细）
        new_pose = self._match(pts, self.grid, self.pose)
        if new_pose is None:
            return
        prev = self.pose_prev if self.pose_prev is not None else self.pose
        moved = np.hypot(new_pose[0] - prev[0], new_pose[1] - prev[1])
        dyaw = abs(normalize_angle(new_pose[2] - prev[2]))
        # 跳变保护：单帧位移/转角过大视为匹配失败，沿用上一帧位姿
        if moved > 0.6 or dyaw > 0.45:
            self.get_logger().warn('匹配跳变被抑制 (d=%.2f yaw=%.2f)' % (moved, dyaw))
            return
        self.pose_prev = new_pose.copy()
        self.pose = new_pose
        # 耗时日志（每 5 秒一次，便于观察匹配性能）
        if t0 - self._t_log > 5.0:
            self._t_log = t0
            self.get_logger().info('匹配耗时 %.0fms' % ((time.time() - t0) * 1000))
        # 运动门控：只有实际移动/转动足够才把新帧并入地图。
        # 静止时不叠加（消除重复登记导致的墙重影/漂移）
        if moved > ADD_MOVE or dyaw > ADD_YAW * math.pi / 180:
            self._add_scan(pts, new_pose)
            self.last_add = t0

    def _match(self, pts, grid, pose):
        """栅格相关匹配：多分辨率搜索位姿增量。返回新位姿 (x,y,yaw) 或 None。"""
        x, y, yaw = pose
        best_pose = pose.copy()
        best_score = -1.0
        # 粗搜索：yaw ±28°(步2.3°)，xy ±0.3m(步0.15m) —— 控制候选总数保证实时性
        for dyaw in np.arange(-0.28, 0.281, 0.04):
            c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw)
            rot = pts @ np.array([[c, -s], [s, c]], dtype=np.float32)  # (N,2)
            for ddx in np.arange(-0.30, 0.301, 0.15):
                for ddy in np.arange(-0.30, 0.301, 0.15):
                    sc = self._score(rot, x + ddx, y + ddy)
                    if sc > best_score:
                        best_score = sc
                        best_pose = np.array([x + ddx, y + ddy, yaw + dyaw])
        if best_score < 0:
            return None
        # 精搜索两轮（7³ 候选/轮，兼顾精度与实时）
        for step, span in ((0.015, 0.045), (0.003, 0.009)):
            bx, by, byaw = best_pose
            c, s = math.cos(byaw), math.sin(byaw)
            rot = pts @ np.array([[c, -s], [s, c]], dtype=np.float32)
            for dyaw in np.arange(-span, span + step, step):
                for ddx in np.arange(-span, span + step, step):
                    for ddy in np.arange(-span, span + step, step):
                        sc = self._score(rot, bx + ddx, by + ddy)
                        if sc > best_score:
                            best_score = sc
                            best_pose = np.array([bx + ddx, by + ddy, byaw + dyaw])
        if best_score < SCORE_MIN:
            return None
        return best_pose

    def _score(self, rot, tx, ty):
        """评分 = 占用命中比例 - 0.8×自由区命中比例。
        自由区为负值（与未知区分），匹配滑入自由区会显著扣分，
        从而把位姿锁定在墙的正确位置，抑制沿墙漂移。"""
        ix = ((rot[:, 0] + tx) / RES + HALF).astype(np.int32)
        iy = ((rot[:, 1] + ty) / RES + HALF).astype(np.int32)
        valid = (ix >= 0) & (ix < GRID) & (iy >= 0) & (iy < GRID)
        if valid.sum() < 60:
            return -1.0
        occ = self.grid[iy[valid], ix[valid]]
        occ_ratio = float((occ > 0.4).mean())
        free_ratio = float((occ < -0.15).mean())
        return occ_ratio - 0.8 * free_ratio

    def _add_scan(self, pts, pose):
        """用 Bresenham 射线更新栅格：端点=占用，射线路径=自由。"""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rx = pose[0] / RES + HALF
        ry = pose[1] / RES + HALF
        step = RES * 0.6
        for p in pts:
            x0 = rx
            y0 = ry
            wx, wy = p[0], p[1]
            dist = math.hypot(wx, wy)
            nx = wx / dist * step / RES if dist > 0 else 0.0
            ny = wy / dist * step / RES if dist > 0 else 0.0
            x1 = (pose[0] + c * wx - s * wy) / RES + HALF
            y1 = (pose[1] + s * wx + c * wy) / RES + HALF
            # 直线插值
            d = math.hypot(x1 - x0, y1 - y0)
            n = max(int(d), 1)
            dx = (x1 - x0) / n
            dy = (y1 - y0) / n
            for i in range(n):
                ix, iy = int(x0), int(y0)
                if 0 <= ix < GRID and 0 <= iy < GRID:
                    # 自由区为负值（下限 -1），与未知区(0)区分，供匹配评分惩罚
                    self.grid[iy, ix] = max(-1.0, self.grid[iy, ix] + LOG_FREE)
                x0 += dx
                y0 += dy
            ix, iy = int(x1), int(y1)
            if 0 <= ix < GRID and 0 <= iy < GRID:
                self.grid[iy, ix] = min(1.0, self.grid[iy, ix] + LOG_HIT)

    # ---------- 输出 ----------

    def publish_map(self):
        with self.lock:
            if not self.has_map:
                return
            grid = self.grid.copy()
            px, py, pyaw = self.pose.copy()
        # 占用量化: >0.55 占用, 负值=自由, 0=未知
        data = np.full(GRID * GRID, -1, dtype=np.int8)
        occ_mask = grid > 0.55
        free_mask = (grid < 0.20) & (grid > 0.001)
        data[occ_mask.ravel()] = 100
        data[free_mask.ravel()] = 0
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.info.resolution = RES
        m.info.width = GRID
        m.info.height = GRID
        m.info.origin.position.x = -HALF * RES
        m.info.origin.position.y = -HALF * RES
        m.data = data.tolist()
        self.pub_map.publish(m)
        # 位姿
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.orientation.z = math.sin(pyaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(pyaw / 2.0)
        self.pub_pose.publish(odom)


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def pointcloud2_to_xyz(msg, zmin, zmax, max_pts, max_xy_range=30.0):
    """从 PointCloud2 提取三维点 (x,y,z)，按 z 带过滤并均匀抽稀。

    ``max_xy_range`` 用于局部实时点云的异常距离保护；传入 ``None``
    时保留全部 XY 范围，例如已处于 map 坐标系的全局地图。
    """
    try:
        offs = {f.name: f.offset for f in msg.fields}
        if 'x' not in offs or 'y' not in offs:
            return None
        ox, oy = offs['x'], offs['y']
        oz = offs.get('z', -1)
        step = msg.point_step or 16
        n = msg.width * msg.height
        if n == 0:
            return None
        data = msg.data
        stride = max(1, n // max_pts)
        pts = []
        for i in range(0, n, stride):
            off = i * step
            z = 0.0
            if oz >= 0:
                z = struct_unpack_f(data, off + oz)
            if z < zmin or z > zmax:
                continue
            x = struct_unpack_f(data, off + ox)
            y = struct_unpack_f(data, off + oy)
            in_xy_range = (max_xy_range is None or
                           math.hypot(x, y) < max_xy_range)
            if (math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and
                    in_xy_range):
                pts.append((x, y, z))
        if len(pts) < 30:
            return None
        return np.array(pts, dtype=np.float32)
    except Exception:
        return None


def pointcloud2_to_xy(msg, zmin, zmax, max_pts):
    """从 PointCloud2 提取二维点 (x,y)，按 z 带过滤并均匀抽稀。"""
    try:
        offs = {f.name: f.offset for f in msg.fields}
        if 'x' not in offs or 'y' not in offs:
            return None
        ox, oy = offs['x'], offs['y']
        oz = offs.get('z', -1)
        step = msg.point_step or 16
        n = msg.width * msg.height
        if n == 0:
            return None
        data = msg.data
        stride = max(1, n // max_pts)
        pts = []
        for i in range(0, n, stride):
            off = i * step
            z = 0.0
            if oz >= 0:
                z = struct_unpack_f(data, off + oz)
            if z < zmin or z > zmax:
                continue
            x = struct_unpack_f(data, off + ox)
            y = struct_unpack_f(data, off + oy)
            if math.isfinite(x) and math.isfinite(y) and math.hypot(x, y) < 30.0:
                pts.append((x, y))
        if len(pts) < 30:
            return None
        return np.array(pts, dtype=np.float32)
    except Exception:
        return None


import struct as _struct


def struct_unpack_f(buf, off):
    return _struct.unpack_from('<f', buf, off)[0]


def main(args=None):
    rclpy.init(args=args)
    node = FallbackSlam()
    # 用单线程执行器：Foxy 的 MultiThreadedExecutor 有已知问题——
    # 高频点云回调会饿死低频回调（复位服务/订阅永不执行）。
    # 单线程下复位回调最多延迟数百毫秒，可接受。
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
