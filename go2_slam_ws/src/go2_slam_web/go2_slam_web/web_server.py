#!/usr/bin/env python3
"""go2_slam_web — 浏览器 LIO-SAM 建图控制台（tornado + rclpy）。

数据源（host Foxy 直接订阅容器 Humble 发布的话题）:
  /lio_sam/mapping/map_global PointCloud2 1Hz  回环修正后的全局地图
  /lio_sam/mapping/odometry   Odometry  ~5Hz   LIO 位姿
  /go2_slam/lidar_preview PointCloud2 ~10Hz C++ 抽稀后的实时点云（雷达系）

地图画面 = 点云投影的 2D 占用栅格, 固定 north-up（map 系）,
机器人只是画布上一个按 yaw 旋转的箭头 —— 图绝不跟着机器人转。
雷达画面 = 车体系, 随狗转向（传感器画面）。

路由:
  GET  /                  前端页面
  GET  /api/status         状态 JSON
  POST /api/save           保存当前回环修正后的三维地图到 maps 目录
  GET  /api/maps           已保存地图文件列表
  GET  /api/download?name= 下载
  WS   /ws                 实时推流 (5Hz)：地图PNG + 雷达点 + 位姿 + 状态
"""

import base64
import json
import math
import os
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import uuid
import zlib

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from unitree_go.msg import LowState

import tornado.ioloop
import tornado.gen
import tornado.web
import tornado.websocket

from go2_slam_core.fallback_slam import pointcloud2_to_xyz
from go2_remembr import RemembrService, UnavailableRemembrService
from go2_slam_web.map_registration import AutoMapRegistration
from go2_slam_web.navigation import CameraBridge, NavigationState, OperationManager

PORT = 8890
MAPS_DIR = '/home/unitree/go2_slam_ws/maps'
INDEX_PATH = os.path.join(os.path.dirname(__file__), 'static', 'index.html')
MAPPING_RESET_SCRIPT = '/home/unitree/go2_slam_ws/reset_mapping.sh'

MAX_LIDAR_PTS = 1400          # 每帧推送给浏览器的雷达点数上限
BROADCAST_HZ = 5.0            # WS 推送频率（足以匹配 LIO 位姿输出）
MAP_PUSH_HZ = 1.0             # 地图 PNG/3D 推送频率
MAX_3D_PTS = 40000            # 3D 视图每帧点数上限（均匀抽稀）
MAX_MAP_PTS = 300000          # 保存/渲染接收的全局地图点上限
MAX_NAV_RAW_OBSTACLE_PTS = 4000       # Web 原始实时占据层抽样上限
MAX_NAV_INFLATED_OBSTACLE_PTS = 8000  # Web 碰撞膨胀层抽样上限
NAV_OBSTACLE_CAPTURE_HZ = 2.0         # 与 SCAN 的低频可视化层匹配，不影响内部规划
MAX_BUILDING_Z = 50.0          # 楼梯/多楼层建图的合理绝对高度保护
MAX_BUILDING_TILT = 1.1        # 允许陡楼梯，仍拒绝接近翻转的异常姿态
Z3D_MAP = (-MAX_BUILDING_Z, MAX_BUILDING_Z)  # 3D/保存保留完整楼层高度

# ---- 2D 栅格化参数 ----
RES = 0.05                    # 栅格分辨率 (m/px)
MAP_MAX_PX = 800              # PNG 最大边长（超了降采样）
Z_MAP = (-0.3, 1.5)           # 建图层 z 带（墙/障碍轮廓）
Z_GROUND = (-1.0, 0.05)       # 地面层 z 带

# 地图配色（RGBA）
C_OCC = (0, 123, 192, 255)        # 占用：Bosch Blue
C_UNKNOWN = (17, 21, 25, 255)     # 地图画布：深色高对比背景

# Browser teleop sends normalized axes only.  The server owns the real speed
# limits and the chassis gate independently clamps them a second time.
TELEOP_VX = 0.40
TELEOP_VY = 0.10
TELEOP_VYAW = 0.45


# ---------------- PNG 编码（纯标准库） ----------------

def encode_png(w, h, rgba):
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw.extend(rgba[y * w * 4:(y + 1) * w * 4])
    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        return c + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) +
            chunk(b'IDAT', zlib.compress(bytes(raw))) + chunk(b'IEND', b''))


def points_to_png(map_pts):
    """map 系点云 -> 固定 north-up 占用栅格 PNG。
    返回 (png, meta) 或 None。meta = {w,h(显示尺寸), res, ox, oy, src_w, src_h(栅格尺寸)}"""
    if len(map_pts) == 0:
        return None
    xmin = min(0.0, float(map_pts[:, 0].min())) - 0.5
    xmax = max(0.0, float(map_pts[:, 0].max())) + 0.5
    ymin = min(0.0, float(map_pts[:, 1].min())) - 0.5
    ymax = max(0.0, float(map_pts[:, 1].max())) + 0.5
    w = max(4, int(math.ceil((xmax - xmin) / RES)))
    h = max(4, int(math.ceil((ymax - ymin) / RES)))

    occ = np.zeros((h, w), np.uint16)
    ix = ((map_pts[:, 0] - xmin) / RES).astype(np.int64)
    iy = ((map_pts[:, 1] - ymin) / RES).astype(np.int64)
    np.clip(ix, 0, w - 1, out=ix)
    np.clip(iy, 0, h - 1, out=iy)
    np.add.at(occ, (iy, ix), 1)

    # north-up：PNG 第 0 行 = 地图 y 最大处
    occ = occ[::-1]

    # 降采样（最大池化保留占用信息）
    k = max(1, max(w, h) // MAP_MAX_PX)
    nw, nh = w // k, h // k
    if k > 1:
        oh, ow = nh * k, nw * k
        occ = occ[:oh, :ow].reshape(nh, k, nw, k).max(axis=(1, 3))

    rgba = np.zeros((nh, nw, 4), np.uint8)
    occ_b = occ > 0
    rgba[:, :] = C_UNKNOWN
    rgba[occ_b] = C_OCC

    # If maximum pooling is used, one displayed pixel spans k native cells.
    # Cropping happens at the south edge after north-up reversal.
    meta = {'w': nw, 'h': nh, 'res': RES * k, 'ox': xmin,
            'oy': ymin + (h - nh * k) * RES,
            'src_w': w, 'src_h': h}
    return encode_png(nw, nh, rgba.tobytes()), meta


def euler_rpy(qx, qy, qz, qw):
    roll = math.atan2(2.0 * (qw * qx + qy * qz),
                      1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (qx * qy + qz * qw),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


# ---------------- 离线地图管理/预览 ----------------

NAVIGATION_SOURCE_MAP = 'map_20260811_155640_273.npz'
MAX_UPLOAD_BYTES = 100 * 1024 * 1024
MAP_EDITOR_MIN_HEIGHT = 0.08
MAP_EDITOR_MAX_HEIGHT = 2.20
NAV_DIR = '/home/unitree/go2_slam_ws/maps/navigation'
NAV_STEM = 'map_20260811_155640_273_nav'
NAV_META_PATH = os.path.join(NAV_DIR, NAV_STEM + '.json')
NAV_PREPARE = '/home/unitree/go2_slam_ws/tools/prepare_navigation_map.py'


def valid_map_name(name):
    return (isinstance(name, str) and 1 < len(name) <= 128 and not name.startswith('.') and
            name == name.strip() and name == os.path.basename(name) and
            '/' not in name and '\\' not in name and name.lower().endswith('.npz') and
            not any(ord(char) < 32 for char in name) and name not in ('.npz', '..npz'))


def map_file_path(name):
    if not valid_map_name(name):
        return None
    path = os.path.abspath(os.path.join(MAPS_DIR, name))
    if os.path.dirname(path) != os.path.abspath(MAPS_DIR):
        return None
    return path


def load_saved_map(path):
    """Load and validate a saved NPZ map without pickle/object arrays."""
    with np.load(path, allow_pickle=False) as data:
        if 'map' not in data:
            raise ValueError('文件中缺少 map 点云')
        points = np.asarray(data['map'], dtype=np.float32)
        ground = (np.asarray(data['ground'], dtype=np.float32)
                  if 'ground' in data else np.empty((0, 3), np.float32))
    if points.ndim != 2 or points.shape[1] < 3 or len(points) == 0:
        raise ValueError('map 必须是非空 N×3 点云')
    points = points[:, :3]
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('点云不包含有限坐标')
    if ground.ndim != 2 or ground.shape[1] < 3:
        ground = np.empty((0, 3), np.float32)
    else:
        ground = ground[:, :3]
        ground = ground[np.isfinite(ground).all(axis=1)]
    return points, ground


def fit_saved_floor_plane(points, ground):
    """Estimate z=ax+by+c from the saved floor band without trusting its median."""
    candidates = ground if len(ground) >= 100 else points[
        points[:, 2] <= np.percentile(points[:, 2], 20.0)]
    if len(candidates) < 20:
        return np.array([0.0, 0.0, float(np.percentile(points[:, 2], 8.0))])
    # A bounded deterministic sample keeps previews responsive on large maps.
    if len(candidates) > 10000:
        candidates = candidates[::int(math.ceil(len(candidates) / 10000.0))]
    design = np.column_stack((candidates[:, :2], np.ones(len(candidates))))
    rng = np.random.RandomState(20260812)
    best = None
    best_count = 0
    for _ in range(300):
        ids = rng.choice(len(candidates), 3, replace=False)
        sample = design[ids]
        if abs(np.linalg.det(sample)) < 1e-5:
            continue
        coeff = np.linalg.solve(sample, candidates[ids, 2])
        if math.hypot(coeff[0], coeff[1]) > math.tan(math.radians(5.0)):
            continue
        inliers = np.abs(candidates[:, 2] - design.dot(coeff)) <= 0.06
        count = int(inliers.sum())
        if count > best_count:
            best, best_count = inliers, count
    if best is None or best_count < 20:
        return np.array([0.0, 0.0, float(np.median(candidates[:, 2]))])
    coeff = np.linalg.lstsq(design[best], candidates[best, 2], rcond=None)[0]
    for _ in range(2):
        residual = np.abs(candidates[:, 2] - design.dot(coeff))
        inliers = residual <= 0.06
        if int(inliers.sum()) < 20:
            break
        coeff = np.linalg.lstsq(design[inliers], candidates[inliers, 2], rcond=None)[0]
    return coeff.astype(np.float64)


def navigation_source_map_name():
    """Current navigation source; kept for backward-compatible protection."""
    try:
        with open(NAV_META_PATH, 'r', encoding='utf-8') as stream:
            source = json.load(stream).get('source', '')
        name = os.path.basename(source)
        return name if valid_map_name(name) else NAVIGATION_SOURCE_MAP
    except (OSError, ValueError, TypeError):
        return NAVIGATION_SOURCE_MAP


def erase_saved_map_regions(source_path, output_path, regions):
    """Erase mid-height points inside multiple XY polygons; preserve floor/roof."""
    points, ground = load_saved_map(source_path)
    if not isinstance(regions, (list, tuple)) or not regions:
        raise ValueError('请至少绘制一个擦除区域')
    if len(regions) > 50:
        raise ValueError('一次最多支持 50 个擦除区域')

    normalized = []
    total_area = 0.0
    total_vertices = 0
    for region in regions:
        raw_vertices = region.get('points') if isinstance(region, dict) else None
        if raw_vertices is None and isinstance(region, dict):
            # Backward compatibility for pages opened before polygon editing.
            xmin, xmax = sorted((float(region['xmin']), float(region['xmax'])))
            ymin, ymax = sorted((float(region['ymin']), float(region['ymax'])))
            raw_vertices = [
                [xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax],
            ]
        if not isinstance(raw_vertices, (list, tuple)):
            raise ValueError('第 %d 个区域缺少轮廓点' % (len(normalized) + 1))
        vertices = []
        for raw_vertex in raw_vertices:
            if isinstance(raw_vertex, dict):
                vertex = (float(raw_vertex['x']), float(raw_vertex['y']))
            else:
                vertex = (float(raw_vertex[0]), float(raw_vertex[1]))
            if not all(math.isfinite(value) for value in vertex):
                raise ValueError('擦除区域坐标无效')
            if not vertices or math.hypot(
                    vertex[0] - vertices[-1][0],
                    vertex[1] - vertices[-1][1]) >= 0.01:
                vertices.append(vertex)
        if len(vertices) > 3 and math.hypot(
                vertices[0][0] - vertices[-1][0],
                vertices[0][1] - vertices[-1][1]) < 0.01:
            vertices.pop()
        if len(vertices) < 3:
            raise ValueError('第 %d 个不规则区域至少需要 3 个轮廓点' % (
                len(normalized) + 1))
        if len(vertices) > 500:
            raise ValueError('单个不规则区域最多支持 500 个轮廓点')
        total_vertices += len(vertices)
        area = 0.5 * abs(sum(
            vertices[index][0] * vertices[(index + 1) % len(vertices)][1] -
            vertices[(index + 1) % len(vertices)][0] * vertices[index][1]
            for index in range(len(vertices))))
        if area < 0.0025:
            raise ValueError('第 %d 个不规则区域面积太小' % (len(normalized) + 1))
        total_area += area
        normalized.append(vertices)
    if total_vertices > 5000:
        raise ValueError('全部不规则区域的轮廓点不能超过 5000 个')
    plane = fit_saved_floor_plane(points, ground)

    relative_z = points[:, 2] - (
        plane[0] * points[:, 0] + plane[1] * points[:, 1] + plane[2])
    inside_xy = np.zeros(len(points), dtype=bool)
    px, py = points[:, 0], points[:, 1]
    for vertices in normalized:
        polygon_inside = np.zeros(len(points), dtype=bool)
        previous_x, previous_y = vertices[-1]
        for current_x, current_y in vertices:
            crosses_y = ((current_y > py) != (previous_y > py))
            edge_x = ((previous_x - current_x) * (py - current_y) /
                      (previous_y - current_y + 1e-15) + current_x)
            polygon_inside ^= crosses_y & (px < edge_x)
            previous_x, previous_y = current_x, current_y
        inside_xy |= polygon_inside
    remove_map = (inside_xy & (relative_z >= MAP_EDITOR_MIN_HEIGHT) &
                  (relative_z <= MAP_EDITOR_MAX_HEIGHT))
    removed = int(remove_map.sum())
    if not removed:
        raise ValueError('框内没有处于可擦除高度的点')
    kept = points[~remove_map]
    # ``ground`` is mapper-provided floor evidence used to fit and fill free
    # space.  Preserve it byte-for-byte even if some rows have global-plane
    # residuals in the editable height band; people are removed from ``map``.
    kept_ground = ground.copy()
    if len(kept) < 100:
        raise ValueError('擦除后地图点数异常，操作已取消')
    # These checks make the height guard an invariant rather than a UI promise.
    floor_before = int((relative_z < MAP_EDITOR_MIN_HEIGHT).sum())
    roof_before = int((relative_z > MAP_EDITOR_MAX_HEIGHT).sum())
    kept_relative_z = relative_z[~remove_map]
    if (int((kept_relative_z < MAP_EDITOR_MIN_HEIGHT).sum()) != floor_before or
            int((kept_relative_z > MAP_EDITOR_MAX_HEIGHT).sum()) != roof_before):
        raise RuntimeError('地面/楼顶保护校验失败')

    temporary = os.path.join(
        os.path.dirname(output_path), '.edit-%s.npz' % uuid.uuid4().hex)
    try:
        np.savez_compressed(temporary, map=kept, ground=kept_ground)
        load_saved_map(temporary)
        os.replace(temporary, output_path)
    finally:
        try:
            os.unlink(temporary)
        except OSError:
            pass
    return {
        'removed_points': removed,
        'remaining_points': int(len(kept)),
        'removed_ground_entries': 0,
        'protected_saved_ground_points': int(len(ground)),
        'protected_floor_points': floor_before,
        'protected_roof_points': roof_before,
        'floor_plane_z_ax_by_c': [float(value) for value in plane],
        'height_range_m': [MAP_EDITOR_MIN_HEIGHT, MAP_EDITOR_MAX_HEIGHT],
        'region_count': int(len(normalized)),
        'selected_area_m2': float(total_area),
        'region_vertex_counts': [int(len(vertices)) for vertices in normalized],
    }


def erase_saved_map_region(source_path, output_path, bounds):
    """Backward-compatible single-box wrapper used by maintenance scripts."""
    return erase_saved_map_regions(source_path, output_path, [bounds])


def rebuild_active_navigation(source_path):
    """Build canonical navigation files in isolation, then promote them."""
    clear_start = [-0.05, 0.21, 0.80]
    try:
        with open(NAV_META_PATH, 'r', encoding='utf-8') as stream:
            old_meta = json.load(stream)
        configured = old_meta.get('cleared_start_xy_radius_m', clear_start)
        if len(configured) == 3:
            clear_start = [float(value) for value in configured]
    except (OSError, ValueError, TypeError):
        pass
    temporary_dir = tempfile.mkdtemp(prefix='.navigation-edit-', dir=NAV_DIR)
    names = [NAV_STEM + suffix for suffix in (
        '.pgm', '.yaml', '_inflated.pgm', '_inflated.yaml',
        '_obstacles.pcd', '.json')]
    try:
        command = [
            NAV_PREPARE, source_path, '--output-dir', temporary_dir,
            '--prefix', NAV_STEM,
            '--clear-start-x', str(clear_start[0]),
            '--clear-start-y', str(clear_start[1]),
            '--clear-start-radius', str(clear_start[2]),
        ]
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=90.0)
        if result.returncode != 0:
            raise RuntimeError(result.stdout.strip() or '导航地图生成失败')
        for name in names:
            if not os.path.isfile(os.path.join(temporary_dir, name)):
                raise RuntimeError('导航地图生成不完整: ' + name)

        metadata_path = os.path.join(temporary_dir, NAV_STEM + '.json')
        with open(metadata_path, 'r', encoding='utf-8') as stream:
            metadata = json.load(stream)
        start_cells = metadata.get('free_connectivity', {}).get(
            'inflated', {}).get('start_component_cells', 0)
        if int(start_cells or 0) <= 0:
            raise RuntimeError(
                '候选地图不覆盖当前导航起点，或起点附近没有膨胀后的可通行区')
        metadata['artifacts'] = {
            'map_yaml': os.path.join(NAV_DIR, NAV_STEM + '.yaml'),
            'inflated_map_yaml': os.path.join(NAV_DIR, NAV_STEM + '_inflated.yaml'),
            'obstacle_pcd': os.path.join(NAV_DIR, NAV_STEM + '_obstacles.pcd'),
        }
        with open(metadata_path, 'w', encoding='utf-8') as stream:
            json.dump(metadata, stream, indent=2, ensure_ascii=False)
            stream.write('\n')

        stamp = time.strftime('%Y%m%d_%H%M%S')
        backup_dir = os.path.join(
            NAV_DIR, 'backups', stamp + '_' + uuid.uuid4().hex[:6] +
            '_before_navigation_change')
        os.makedirs(backup_dir, exist_ok=True)
        for name in names:
            current = os.path.join(NAV_DIR, name)
            if os.path.isfile(current):
                shutil.copy2(current, os.path.join(backup_dir, name))
        staged = []
        for name in names:
            hidden = os.path.join(NAV_DIR, '.promote-%s-%s' % (uuid.uuid4().hex, name))
            shutil.copy2(os.path.join(temporary_dir, name), hidden)
            staged.append((hidden, os.path.join(NAV_DIR, name)))
        # Publish metadata last so readers never see it before its artifacts.
        try:
            for hidden, destination in sorted(
                    staged, key=lambda pair: pair[1].endswith('.json')):
                os.replace(hidden, destination)
        except Exception:
            restore_navigation_backup(backup_dir)
            raise
        return metadata, backup_dir
    finally:
        shutil.rmtree(temporary_dir, ignore_errors=True)


def restore_navigation_backup(backup_dir):
    """Restore a complete canonical navigation artifact set."""
    names = [NAV_STEM + suffix for suffix in (
        '.pgm', '.yaml', '_inflated.pgm', '_inflated.yaml',
        '_obstacles.pcd', '.json')]
    for name in names:
        source = os.path.join(backup_dir, name)
        if os.path.isfile(source):
            temporary = os.path.join(
                NAV_DIR, '.restore-%s-%s' % (uuid.uuid4().hex, name))
            shutil.copy2(source, temporary)
            os.replace(temporary, os.path.join(NAV_DIR, name))


def switch_navigation_baseline(navigation, source_path):
    """Build, promote and load a selected map with automatic rollback."""
    metadata, backup_dir = rebuild_active_navigation(source_path)
    try:
        navigation.reload_map()
    except Exception:
        restore_navigation_backup(backup_dir)
        navigation.reload_map()
        raise
    return metadata, backup_dir


def build_saved_map_preview(path):
    points, ground = load_saved_map(path)
    plane = fit_saved_floor_plane(points, ground)
    floor_z = float(plane[2])
    leveled = points.copy()
    leveled[:, 2] -= (plane[0] * leveled[:, 0] +
                      plane[1] * leveled[:, 1] + plane[2])

    z = leveled[:, 2]
    cloud3d = leveled[(z >= -0.12) & (z <= 3.5)]
    if not len(cloud3d):
        cloud3d = leveled
    if len(cloud3d) > 70000:
        cloud3d = cloud3d[::int(math.ceil(len(cloud3d) / 70000.0))]

    obstacles = leveled[(z >= 0.08) & (z <= 2.2)]
    if len(obstacles) < 10:
        obstacles = leveled
    rendered = points_to_png(obstacles[:, :2].astype(np.float64, copy=False))
    if not rendered:
        raise ValueError('无法生成 2D 预览')
    png, meta = rendered
    return {
        'map_png': 'data:image/png;base64,' + base64.b64encode(png).decode(),
        'map': meta,
        'map3d': WebNode._downsample_3d(cloud3d, 70000),
        'point_count': int(len(points)),
        'preview_point_count': int(len(cloud3d)),
        'floor_z': round(floor_z, 3),
        'floor_plane': [round(float(value), 7) for value in plane],
        'erase_height_range': [MAP_EDITOR_MIN_HEIGHT, MAP_EDITOR_MAX_HEIGHT],
    }


# ---------------- ROS2 数据节点 ----------------

class WebNode(Node):

    def __init__(self, navigation):
        super().__init__('web_server')
        self.navigation = navigation
        # 雷达安装校准：雷达前向相对狗前向的偏航角（度），与 fallback_slam 保持一致
        self.declare_parameter('lidar_yaw_offset', -90.0)
        self._yaw_off = math.radians(self.get_parameter('lidar_yaw_offset').value)
        self.lock = threading.Lock()
        self.map_png = None          # base64 data-url
        self.map_meta = None
        self.map_t = 0.0             # 最后一次收到地图的时间
        self.map_pts_all = None      # 全量点云（保存地图用）
        self.map_full_pts = None     # 保留 z 的全局地图（保存用）
        self.ground_pts_all = None
        self.map3d_pts = None        # 3D 视图点（map 系, 含 z）
        self.ground3d_pts = None
        self.map_stats = {'pts': 0, 'range': '--'}
        self.pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 't': 0.0}
        self.pose_t = 0.0
        self.pose_valid = False
        self.pose_error = '等待 LIO 里程计'
        self._last_raw_pose = None
        self.lidar_pts = []
        self.lidar3d_pts = []      # 3D 实时点云（狗系），3D 视图白色参照
        self.lidar_rate = 0.0
        self.lidar_n = 0
        self.lidar_t0 = time.time()
        self._stream_lock = threading.Lock()
        self._map_sub = None
        self._lidar_sub = None
        self._registration_lock = threading.Lock()
        self._registration_active = False
        self._registration_clouds = []
        self._registration_sub = None

        # 建图页默认关闭；由 Web 上的“开始建图”按钮显式启用大点云订阅。
        self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.on_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/scan_planner/body_pose', self.on_navigation_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            Path, '/scan_planner/local_path', self.navigation.update_local_path, 10)
        self.create_subscription(
            Path, '/scan_planner/planning/local_horizon',
            self.navigation.update_local_horizon, qos_profile_sensor_data)
        self._nav_obstacle_last = {'raw': 0.0, 'inflated': 0.0}
        self.create_subscription(
            PointCloud2, '/scan_planner/grid_map/occupancy',
            self.on_navigation_obstacles, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/scan_planner/grid_map/occupancy_inflate',
            self.on_navigation_obstacles_inflated, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/scan_planner/planning/local_waiting',
            self.navigation.update_local_waiting, 10)
        self.create_subscription(
            String, '/scan_planner/recovery_status',
            self.navigation.update_recovery_status, 10)
        # 机身自带 IMU/电池状态；不使用摄像头附带的 /utlidar/imu。
        # Web UI does not need the 200 Hz control-bus stream.  The robot also
        # exposes the same LowState payload on /lf/lowstate at about 20 Hz;
        # using it avoids a full CPU core of Python DDS deserialization while
        # keeping battery and attitude display responsive.
        self.create_subscription(
            LowState, '/lf/lowstate', self.navigation.update_lowstate,
            qos_profile_sensor_data)
        self.global_path_pub = self.create_publisher(
            Path, '/scan_planner/global_path', 1)
        self.navigation_cancel_pub = self.create_publisher(
            Bool, '/scan_planner/cancel', 10)
        self.chassis_enable_pub = self.create_publisher(
            Bool, '/scan_planner/chassis_enable', 10)
        self.chassis_heartbeat_pub = self.create_publisher(
            Bool, '/scan_planner/chassis_heartbeat', 10)
        self.teleop_enable_pub = self.create_publisher(
            Bool, '/scan_planner/teleop_enable', 10)
        self.teleop_cmd_pub = self.create_publisher(
            Twist, '/scan_planner/teleop_cmd', 10)
        self.posture_command_pub = self.create_publisher(
            String, '/scan_planner/posture_command', 10)
        self.alignment_valid_pub = self.create_publisher(
            Bool, '/scan_planner/alignment_valid', 10)
        self.nav2_goal_pub = self.create_publisher(
            PoseStamped, '/go2/nav2/goal', 10)
        self.nav2_cancel_pub = self.create_publisher(
            Bool, '/go2/nav2/cancel', 10)
        self.nav2_map_odom_pub = self.create_publisher(
            TransformStamped, '/go2/nav2/map_to_odom', 10)
        self.navigation_backend_pub = self.create_publisher(
            String, '/go2/navigation/backend', 10)
        self.navigation.chassis_enable_callback = self.publish_chassis_enable
        self.chassis_status = {
            'connected': False, 'enabled': False, 'ready': False,
            'teleop_enabled': False, 'control_mode': 'navigation',
            'navigation_backend': 'nav2',
            'reason': '等待底盘安全门',
        }
        self.chassis_status_t = 0.0
        self.create_subscription(
            String, '/scan_planner/chassis_status', self.on_chassis_status, 10)
        self.create_subscription(
            Bool, '/scan_planner/navigation_completed',
            self.on_scan_navigation_completed, 10)
        self.create_subscription(
            Bool, '/go2/nav2/navigation_completed',
            self.on_nav2_navigation_completed, 10)
        self.nav2_status = {
            'state': 'offline', 'message': 'Nav2 导航服务未连接'}
        self.nav2_status_t = 0.0
        self.nav2_shadow_metrics = {}
        self.create_subscription(
            String, '/go2/nav2/status', self.on_nav2_status, 10)
        self.create_subscription(
            String, '/go2/nav2/shadow_metrics',
            self.on_nav2_shadow_metrics, 10)
        self.planner_commands = {
            'scan': (0.0, 0.0, 0.0),
            'nav2': (0.0, 0.0, 0.0),
        }
        self.planner_command_times = {'scan': 0.0, 'nav2': 0.0}
        self.create_subscription(
            Twist, '/scan_planner/cmd_vel_test',
            lambda msg: self.on_planner_command('scan', msg), 10)
        self.create_subscription(
            Twist, '/go2/nav2/cmd_vel_safe',
            lambda msg: self.on_planner_command('nav2', msg), 10)

        self.create_timer(1.0, self.update_lidar_rate)
        # Safety heartbeat must not share the single-threaded ROS callback
        # queue with odometry, LowState and planner traffic. A burst of those
        # callbacks previously delayed this 5 Hz signal beyond the 1 s safety
        # timeout even though the Web process itself was healthy.
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name='go2-web-safety-heartbeat', daemon=True)
        self._heartbeat_thread.start()

    def set_mapping_streams(self, enabled):
        """Subscribe to the heavy map/preview topics only on the mapping page."""
        with self._stream_lock:
            if enabled:
                if self._map_sub is None:
                    self._map_sub = self.create_subscription(
                        PointCloud2, '/lio_sam/mapping/map_global', self.on_map,
                        qos_profile_sensor_data)
                if self._lidar_sub is None:
                    self._lidar_sub = self.create_subscription(
                        PointCloud2, '/go2_slam/lidar_preview', self.on_lidar,
                        qos_profile_sensor_data)
            else:
                if self._map_sub is not None:
                    self.destroy_subscription(self._map_sub)
                    self._map_sub = None
                if self._lidar_sub is not None:
                    self.destroy_subscription(self._lidar_sub)
                    self._lidar_sub = None

    # ---- 订阅回调 ----

    def on_map(self, msg):
        # 单次解析后按高度切分，避免过去同一大点云重复解码 2~3 次。
        # map_global 的 XY 已是从建图原点累积的全局坐标，不能
        # 复用局部雷达的 30 m 异常距离保护，否则大地图会被裁成圆。
        all_pts = pointcloud2_to_xyz(
            msg, -MAX_BUILDING_Z, MAX_BUILDING_Z,
            MAX_MAP_PTS, max_xy_range=None)
        if all_pts is None or len(all_pts) == 0:
            return
        all_pts = np.asarray(all_pts, dtype=np.float32)
        z = all_pts[:, 2]
        pts3d = all_pts[(z >= Z3D_MAP[0]) & (z <= Z3D_MAP[1])]
        map3d = all_pts[(z >= Z_MAP[0]) & (z <= Z_MAP[1])]
        ground3d = all_pts[(z >= Z_GROUND[0]) & (z <= Z_GROUND[1])]
        if len(map3d) == 0:
            return
        pts = map3d[:, :2].astype(np.float64, copy=False)
        ground = ground3d[:, :2].astype(np.float64, copy=False)
        try:
            rendered = points_to_png(pts)
            if not rendered:
                return
            png, meta = rendered
            png_url = 'data:image/png;base64,' + base64.b64encode(png).decode()
            stats = {
                'pts': len(all_pts),
                'range': 'x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]' % (
                    all_pts[:, 0].min(), all_pts[:, 0].max(),
                    all_pts[:, 1].min(), all_pts[:, 1].max(),
                    all_pts[:, 2].min(), all_pts[:, 2].max()),
            }
            # 栅格化和 PNG 压缩可能较慢，全部在锁外完成；锁内只替换快照。
            with self.lock:
                self.map_pts_all = pts
                self.map_full_pts = all_pts
                self.ground_pts_all = ground
                self.map3d_pts = pts3d
                self.ground3d_pts = ground3d
                self.map_png = png_url
                self.map_meta = meta
                self.map_stats = stats
                self.map_t = time.time()
        except Exception as e:
            self.get_logger().error('map render: %s' % e)

    def on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        roll, pitch, yaw = euler_rpy(q.x, q.y, q.z, q.w)
        now = time.time()
        values = (p.position.x, p.position.y, p.position.z, roll, pitch, yaw)
        valid = all(math.isfinite(v) for v in values)
        reason = ''
        if valid and (abs(p.position.z) > MAX_BUILDING_Z or
                      abs(roll) > MAX_BUILDING_TILT or
                      abs(pitch) > MAX_BUILDING_TILT):
            valid = False
            reason = 'LIO 姿态/高度越界'
        raw = (p.position.x, p.position.y, p.position.z, now)
        if valid and self._last_raw_pose is not None:
            prev = self._last_raw_pose
            dt = now - prev[3]
            if dt > 0.02:
                speed = math.sqrt(sum((raw[i] - prev[i]) ** 2 for i in range(3))) / dt
                if speed > 5.0:
                    valid = False
                    reason = 'LIO 位姿跳变 %.1fm/s' % speed
        self._last_raw_pose = raw
        with self.lock:
            self.pose_t = now
            self.pose_valid = valid
            self.pose_error = reason if not valid else ''
            if valid:
                self.pose = {
                    'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                    'yaw': yaw, 't': now,
                }

    def on_lidar(self, msg):
        pts3d = pointcloud2_to_xyz(msg, -1.0, 3.0, MAX_LIDAR_PTS)
        if pts3d is None:
            return
        pts3d = np.asarray(pts3d, dtype=np.float32)
        # 雷达系 -> 狗系：雷达画面"前向朝上"= 狗前向朝上
        # 直接做逐列运算，避免小矩阵乘法唤醒整组 OpenBLAS 工作线程。
        if self._yaw_off:
            c, s = math.cos(self._yaw_off), math.sin(self._yaw_off)
            x = pts3d[:, 0].copy()
            y = pts3d[:, 1].copy()
            pts3d[:, 0] = c * x + s * y
            pts3d[:, 1] = -s * x + c * y
        z_mask = (pts3d[:, 2] >= -0.8) & (pts3d[:, 2] <= 0.8)
        pts = pts3d[z_mask, :2]
        with self.lock:
            self.lidar_pts = pts.tolist()
            self.lidar3d_pts = pts3d.tolist() if pts3d is not None else []
            self.lidar_n += 1

    def on_navigation_pose(self, msg):
        q = msg.pose.pose.orientation
        self.navigation.update_pose(msg, euler_rpy(q.x, q.y, q.z, q.w))

    def _capture_navigation_obstacles(self, msg, inflated):
        layer = 'inflated' if inflated else 'raw'
        now = time.monotonic()
        if now - self._nav_obstacle_last[layer] < 1.0 / NAV_OBSTACLE_CAPTURE_HZ:
            return
        self._nav_obstacle_last[layer] = now
        source_count = int(msg.width * msg.height)
        if source_count == 0:
            points = np.empty((0, 3), np.float32)
            planning_points = points if inflated else None
        elif inflated:
            # Temporary-start selection is safety-relevant, so retain every
            # live inflated voxel for that one-shot query. Continue sending a
            # bounded uniform sample to the browser to preserve bandwidth.
            planning_points = pointcloud2_to_xyz(
                msg, -5.0, 5.0, source_count, max_xy_range=None)
            if planning_points is None:
                return
            stride = max(
                1, int(math.ceil(
                    len(planning_points) / MAX_NAV_INFLATED_OBSTACLE_PTS)))
            points = planning_points[::stride][:MAX_NAV_INFLATED_OBSTACLE_PTS]
        else:
            points = pointcloud2_to_xyz(
                msg, -5.0, 5.0,
                MAX_NAV_RAW_OBSTACLE_PTS,
                max_xy_range=None)
            if points is None:
                return
            planning_points = None
        self.navigation.update_obstacle_cloud(
            points, inflated, source_count, planning_points)

    def on_navigation_obstacles(self, msg):
        self._capture_navigation_obstacles(msg, False)

    def on_navigation_obstacles_inflated(self, msg):
        self._capture_navigation_obstacles(msg, True)

    def on_registration_cloud(self, msg):
        with self._registration_lock:
            if not self._registration_active or len(self._registration_clouds) >= 10:
                return
        points = pointcloud2_to_xyz(msg, -2.0, 4.0, 5000)
        if points is None or len(points) < 200:
            return
        with self._registration_lock:
            if self._registration_active and len(self._registration_clouds) < 10:
                self._registration_clouds.append(np.asarray(points, dtype=np.float32))

    def begin_registration_capture(self):
        with self._registration_lock:
            if self._registration_active:
                return False
            self._registration_clouds = []
            self._registration_active = True
            if self._registration_sub is None:
                self._registration_sub = self.create_subscription(
                    PointCloud2, '/lio_sam/mapping/cloud_registered',
                    self.on_registration_cloud, qos_profile_sensor_data)
            return True

    def registration_capture_count(self):
        with self._registration_lock:
            return len(self._registration_clouds)

    def finish_registration_capture(self):
        with self._registration_lock:
            self._registration_active = False
            clouds = list(self._registration_clouds)
            self._registration_clouds = []
            subscription = self._registration_sub
            self._registration_sub = None
        if subscription is not None:
            self.destroy_subscription(subscription)
        return clouds

    def publish_global_path(self, points):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            if index + 1 < len(points):
                nxt = points[index + 1]
                yaw = math.atan2(nxt[1] - point[1], nxt[0] - point[0])
            elif index:
                prev = points[index - 1]
                yaw = math.atan2(point[1] - prev[1], point[0] - prev[0])
            else:
                yaw = 0.0
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose)
        self.global_path_pub.publish(path)

    def publish_navigation_cancel(self, cancelled):
        message = Bool()
        message.data = bool(cancelled)
        self.navigation_cancel_pub.publish(message)
        self.nav2_cancel_pub.publish(message)

    def publish_nav2_goal(self, x, y):
        values = (float(x), float(y))
        if not all(math.isfinite(value) for value in values):
            return False
        message = PoseStamped()
        message.header.frame_id = 'nav_map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = values[0]
        message.pose.position.y = values[1]
        message.pose.orientation.w = 1.0
        self.nav2_goal_pub.publish(message)
        return True

    def publish_chassis_enable(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.chassis_enable_pub.publish(message)

    def publish_navigation_backend(self, backend):
        backend = str(backend).strip().lower()
        if backend not in ('scan', 'nav2'):
            return False
        message = String()
        message.data = backend
        self.navigation_backend_pub.publish(message)
        return True

    def publish_teleop_enable(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self.teleop_enable_pub.publish(message)

    def publish_teleop_axes(self, forward, lateral, turn):
        axes = (float(forward), float(lateral), float(turn))
        if not all(math.isfinite(value) for value in axes):
            return False
        forward, lateral, turn = (
            max(-1.0, min(1.0, value)) for value in axes)
        message = Twist()
        message.linear.x = forward * TELEOP_VX
        message.linear.y = lateral * TELEOP_VY
        message.angular.z = turn * TELEOP_VYAW
        self.teleop_cmd_pub.publish(message)
        return True

    def publish_teleop_stop(self):
        self.publish_teleop_axes(0.0, 0.0, 0.0)

    def publish_posture_command(self, action):
        if action not in ('stand_down', 'recovery_stand'):
            return False
        message = String()
        message.data = action
        self.posture_command_pub.publish(message)
        return True

    def publish_chassis_heartbeat(self):
        message = Bool()
        message.data = True
        self.chassis_heartbeat_pub.publish(message)
        alignment = Bool()
        alignment.data = self.navigation.is_alignment_valid()
        self.alignment_valid_pub.publish(alignment)
        transform = self.navigation.nav2_map_to_odom_tf()
        if transform is not None:
            message = TransformStamped()
            message.header.frame_id = 'nav_map'
            message.child_frame_id = 'odom'
            message.header.stamp = self.get_clock().now().to_msg()
            message.transform.translation.x = transform['x']
            message.transform.translation.y = transform['y']
            message.transform.translation.z = transform['z']
            message.transform.rotation.z = math.sin(0.5 * transform['yaw'])
            message.transform.rotation.w = math.cos(0.5 * transform['yaw'])
            self.nav2_map_odom_pub.publish(message)

    def _heartbeat_loop(self):
        period = 0.20
        next_tick = time.monotonic()
        while not self._heartbeat_stop.is_set():
            try:
                self.publish_chassis_heartbeat()
            except Exception as exc:
                if not self._heartbeat_stop.is_set():
                    self.get_logger().error(
                        'Safety heartbeat publication failed: %s' % exc)
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay < 0.0:
                next_tick = time.monotonic()
                delay = 0.0
            self._heartbeat_stop.wait(delay)

    def destroy_node(self):
        self._heartbeat_stop.set()
        heartbeat_thread = getattr(self, '_heartbeat_thread', None)
        if (heartbeat_thread is not None and heartbeat_thread.is_alive() and
                heartbeat_thread is not threading.current_thread()):
            heartbeat_thread.join(timeout=1.0)
        return super().destroy_node()

    def on_chassis_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                was_enabled = bool(self.chassis_status.get('enabled'))
                self.chassis_status = status
                self.chassis_status_t = time.monotonic()
                if (was_enabled and not status.get('enabled') and
                        self.navigation.has_active_goal()):
                    reason = str(status.get('reason') or '底盘安全门已锁定')
                    self.navigation.clear_navigation(
                        reason + '；目标和路径已清除')
                    self.publish_navigation_cancel(True)
                    self.publish_global_path([])
                    self.get_logger().warn(
                        'Chassis gate locked during navigation (%s); '
                        'route cleared' % reason)
        except (TypeError, ValueError):
            pass

    def complete_navigation(self, source):
        selected = self.chassis_status.get('navigation_backend', 'nav2')
        if selected != source:
            self.get_logger().info(
                'Ignored %s completion; selected backend is %s' %
                (source.upper(), selected.upper()))
            return
        # The selected controller is the source of truth for completion. Clear
        # all parallel planners and lock the chassis before accepting a new goal.
        self.navigation.clear_navigation('已到达目标点')
        self.publish_global_path([])
        self.publish_navigation_cancel(True)
        self.publish_chassis_enable(False)
        self.get_logger().info(
            '%s navigation completed; route cleared and chassis locked' %
            source.upper())

    def on_scan_navigation_completed(self, msg):
        if not msg.data:
            return
        self.complete_navigation('scan')

    def on_nav2_navigation_completed(self, msg):
        if not msg.data:
            return
        self.complete_navigation('nav2')

    def on_nav2_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.nav2_status = status
                self.nav2_status_t = time.monotonic()
        except (TypeError, ValueError):
            pass

    def on_nav2_shadow_metrics(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.nav2_shadow_metrics = status
        except (TypeError, ValueError):
            pass

    def on_planner_command(self, backend, msg):
        self.planner_commands[backend] = (
            float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self.planner_command_times[backend] = time.time()

    def update_lidar_rate(self):
        with self.lock:
            now = time.time()
            dt = now - self.lidar_t0
            self.lidar_rate = self.lidar_n / dt if dt > 0 else 0.0
            self.lidar_n = 0
            self.lidar_t0 = now

    def reset_mapping(self):
        """Replace the in-memory LIO instance, then clear every Web map cache."""
        try:
            result = subprocess.run(
                ['/bin/bash', MAPPING_RESET_SCRIPT],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=40.0)
        except subprocess.TimeoutExpired:
            return False, 'LIO-SAM 重启超时；当前建图未确认清除'
        output = result.stdout.strip()
        if result.returncode != 0:
            return False, output or 'LIO-SAM 重启失败，当前建图未确认清除'

        with self.lock:
            self.map_png = None
            self.map_meta = None
            self.map_t = 0.0
            self.map_pts_all = None
            self.map_full_pts = None
            self.ground_pts_all = None
            self.map3d_pts = None
            self.ground3d_pts = None
            self.map_stats = {'pts': 0, 'range': '--'}
            self.pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 't': 0.0}
            self.pose_t = 0.0
            self.pose_valid = False
            self.pose_error = '等待 LIO 重新初始化'
            self._last_raw_pose = None
        return True, output or '当前建图已清除，LIO-SAM 正在重新初始化'

    # ---- 保存地图 ----

    def do_save(self, requested_name=None):
        """保存保留 z 的回环修正后 LIO 全局地图。"""
        with self.lock:
            mp = None if self.map_full_pts is None else self.map_full_pts.copy()
            gr = None if self.ground3d_pts is None else self.ground3d_pts.copy()
            map_age = time.time() - self.map_t if self.map_t else float('inf')
            pose_ok = self.pose_valid and time.time() - self.pose_t < 1.0
        if mp is None or len(mp) == 0:
            return False, '还没有有效地图数据（/lio_sam/mapping/map_global 未收到）'
        if map_age >= 3.0 or not pose_ok:
            return False, 'LIO 地图或位姿已离线，拒绝保存过期数据'
        now = time.time()
        if requested_name:
            name = str(requested_name).strip()
            if not name.lower().endswith('.npz'):
                name += '.npz'
            if not valid_map_name(name):
                return False, '地图名无效，只允许普通 .npz 文件名'
            fn = name
        else:
            fn = 'map_%s_%03d.npz' % (
                time.strftime('%Y%m%d_%H%M%S', time.localtime(now)),
                int(now * 1000) % 1000)
        fp = os.path.join(MAPS_DIR, fn)
        if os.path.exists(fp):
            return False, '地图名已存在，请换一个名称'
        os.makedirs(MAPS_DIR, exist_ok=True)
        np.savez_compressed(
            fp, map=mp,
            ground=(gr if gr is not None else np.empty((0, 3), dtype=np.float32)))
        return True, '已保存 %s (%d 个三维点)' % (fn, len(mp))

    @staticmethod
    def _downsample_3d(pts, max_pts):
        """3D 点云均匀抽稀为扁平 [x,y,z,...] 列表（round 3 减小 JSON）。"""
        if pts is None or len(pts) == 0:
            return []
        n = len(pts)
        stride = max(1, n // max_pts)
        out = []
        append = out.append
        for i in range(0, n, stride):
            p = pts[i]
            append(round(float(p[0]), 3))
            append(round(float(p[1]), 3))
            append(round(float(p[2]), 3))
        return out

    def snapshot(self, with3d=False):
        with self.lock:
            now = time.time()
            map_age = now - self.map_t if self.map_t else None
            pose_age = now - self.pose_t if self.pose_t else None
            map_online = map_age is not None and map_age < 3.0
            pose_online = pose_age is not None and pose_age < 1.0
            healthy = map_online and pose_online and self.pose_valid
            map_png = self.map_png if map_online else None
            return {
                'map_png': map_png,
                'map': self.map_meta if map_online else None,
                # 3D 抽稀开销大（1.2万点 round×3），只在 1Hz map_tick 帧计算
                'map3d': self._downsample_3d(self.map3d_pts, MAX_3D_PTS)
                if with3d and map_online else [],
                # 地面点仅保留给 NPZ 地图保存，界面不再单独标红显示。
                'ground3d': [],
                'pose': dict(self.pose),
                'lidar': [round(v, 3) for pair in self.lidar_pts for v in pair],
                'lidar3d': [round(v, 3) for p in self.lidar3d_pts for v in p],
                'lidar_rate': round(self.lidar_rate, 1),
                'map_stats': dict(self.map_stats),
                'health': {
                    'healthy': healthy,
                    'map_online': map_online,
                    'pose_online': pose_online,
                    'pose_valid': self.pose_valid,
                    'map_age': None if map_age is None else round(map_age, 2),
                    'pose_age': None if pose_age is None else round(pose_age, 2),
                    'error': self.pose_error,
                },
            }


# ---------------- tornado handlers ----------------

class IndexHandler(tornado.web.RequestHandler):
    def get(self):
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.set_header('Pragma', 'no-cache')
        try:
            with open(INDEX_PATH, 'rb') as f:
                page = f.read()
            mode = self.application.operations.mode
            initial = ('<body data-initial-mode="%s">' % mode).encode('utf-8')
            if mode in ('navigation', 'maps'):
                # 首包就按服务端模式渲染，避免浏览器加载脚本前闪过建图页。
                common = (
                    (b'class="mode-tab active" id="tabMapping"',
                     b'class="mode-tab" id="tabMapping"'),
                    (b'<main id="mappingPage">', b'<main id="mappingPage" class="hidden">'),
                    (b'<footer id="mappingFooter">', b'<footer id="mappingFooter" class="hidden">'),
                )
                if mode == 'navigation':
                    specific = (
                        (b'class="mode-tab" id="tabNavigation"',
                         b'class="mode-tab active" id="tabNavigation"'),
                        (b'<h1 id="pageTitle">GO2-W \xe5\xbb\xba\xe5\x9b\xbe\xe6\x8e\xa7\xe5\x88\xb6\xe5\x8f\xb0</h1>',
                         b'<h1 id="pageTitle">GO2-W \xe5\xaf\xbc\xe8\x88\xaa\xe6\x8e\xa7\xe5\x88\xb6\xe5\x8f\xb0</h1>'),
                        (b'<section id="navigationPage" class="nav-page hidden">',
                         b'<section id="navigationPage" class="nav-page">'),
                        (b'<footer id="navigationFooter" class="hidden">',
                         b'<footer id="navigationFooter">'),
                    )
                else:
                    specific = (
                        (b'class="mode-tab" id="tabMaps"',
                         b'class="mode-tab active" id="tabMaps"'),
                        (b'<h1 id="pageTitle">GO2-W \xe5\xbb\xba\xe5\x9b\xbe\xe6\x8e\xa7\xe5\x88\xb6\xe5\x8f\xb0</h1>',
                         b'<h1 id="pageTitle">GO2-W \xe5\x9c\xb0\xe5\x9b\xbe\xe7\xae\xa1\xe7\x90\x86</h1>'),
                        (b'<section id="mapsPage" class="maps-page hidden">',
                         b'<section id="mapsPage" class="maps-page">'),
                    )
                for old, new in common + specific:
                    page = page.replace(old, new, 1)
            self.write(page.replace(b'<body>', initial, 1))
        except Exception:
            self.write('index.html 未找到')


class ApiStatusHandler(tornado.web.RequestHandler):
    def get(self):
        snap = self.application.web_node.snapshot()
        self.write(json.dumps({
            'pose': snap['pose'],
            'lidar_rate': snap['lidar_rate'],
            'map_stats': snap['map_stats'],
            'health': snap['health'],
            'chassis': dict(self.application.web_node.chassis_status),
            'nav2': dict(self.application.web_node.nav2_status),
            'maps_dir': MAPS_DIR,
            'mode': self.application.operations.mode,
        }))


class ApiModeHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({
            'mode': self.application.operations.mode,
            'planner_active': self.application.operations.planner_active(),
            'camera': self.application.camera.status(),
            'mapping_active': self.application.operations.mapping_active,
        }))

    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            target = body.get('mode')
            save_name = body.get('save_name')
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': 'JSON 格式错误'}))
            return
        chassis = self.application.web_node.chassis_status
        if target != 'navigation':
            await tornado.ioloop.IOLoop.current().run_in_executor(
                None, self.application.remembr.set_enabled, False)
        preserve_teleop = bool(
            self.application.teleop_owner and chassis.get('teleop_enabled') and
            chassis.get('enabled'))
        ok, message, mode = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.operations.switch, target, save_name,
            preserve_teleop)
        self.write(json.dumps({'success': ok, 'message': message, 'mode': mode}))


class ApiMappingHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(json.dumps({
            'active': self.application.operations.mapping_active,
            'mode': self.application.operations.mode,
        }))

    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            action = body.get('action')
        except Exception:
            action = None
        if action == 'start':
            callback = self.application.operations.start_mapping
            args = ()
        elif action == 'stop':
            callback = self.application.operations.stop_mapping
            args = (None, False)
        elif action == 'clear':
            callback = self.application.operations.clear_mapping
            args = ()
        else:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '未知建图操作'}))
            return
        result = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, callback, *args)
        self.write(json.dumps({'success': result[0], 'message': result[1],
                               'active': self.application.operations.mapping_active}))


class ApiNavigationMapHandler(tornado.web.RequestHandler):
    def get(self):
        self.set_header('Content-Type', 'application/json; charset=utf-8')
        # The URL is stable while the selected navigation baseline is mutable.
        # Never let browsers reuse the previous baseline's point cloud.
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.set_header('Pragma', 'no-cache')
        self.set_header('Expires', '0')
        self.write(self.application.navigation_map_json)


class ApiNavigationGoalHandler(tornado.web.RequestHandler):
    async def post(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '当前不在导航模式'}))
            return
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            x, y = float(body['x']), float(body['y'])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '目标坐标无效'}))
            return
        if not self.application.navigation.is_alignment_valid():
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '请先完成本次开机后的地图位姿标定',
            }))
            return
        chassis = self.application.web_node.chassis_status
        if chassis.get('teleop_enabled'):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '键盘控制正在占用底盘，请先关闭键盘控制',
            }))
            return
        if not chassis.get('enabled'):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '底盘尚未启用；请先点“启用真实底盘”，'
                           '确认显示“已启用”后再发送目标',
            }))
            return
        if not self.application.planning_lock.acquire(blocking=False):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '已有一个全局路径请求正在计算，请等待其结束',
            }))
            return
        try:
            planner_ok, planner_message = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, self.application.operations.ensure_navigation)
            if not planner_ok:
                self.write(json.dumps({'success': False, 'message': planner_message}))
                return
            ok, message, points = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, self.application.navigation.plan, x, y)
        finally:
            self.application.planning_lock.release()
        if ok:
            self.application.camera.start()
            issued_at = time.time()
            backend = str(chassis.get('navigation_backend', 'nav2'))
            self.application.web_node.publish_navigation_cancel(False)
            adjusted_goal = self.application.navigation.goal
            if backend == 'scan':
                # Legacy standalone mode. In the normal hybrid deployment the
                # Nav2 Controller plugin is the only publisher of SCAN's
                # reference path, preventing two global-path generations from
                # racing on the same topic.
                self.application.web_node.publish_global_path(points)
            elif adjusted_goal is not None:
                self.application.web_node.publish_nav2_goal(
                    adjusted_goal['x'], adjusted_goal['y'])
            # A fresh nonzero safe command confirms that Nav2, the SCAN local
            # trajectory adapter and the safety filters have all responded.
            # A zero result can also be a legitimate local wait, so keep the
            # accepted target and let Nav2's bounded recovery policy decide.
            command_ready = False
            deadline = time.time() + 3.0
            while time.time() < deadline:
                command = self.application.web_node.planner_commands.get(
                    backend, (0.0, 0.0, 0.0))
                command_t = self.application.web_node.planner_command_times.get(
                    backend, 0.0)
                if (command_t > issued_at and
                        max(abs(value) for value in command) > 0.01):
                    command_ready = True
                    break
                await tornado.gen.sleep(0.05)
            if not command_ready:
                message += '；局部轨迹暂不可用，已原地等待并自动重试'
        self.write(json.dumps({'success': ok, 'message': message,
                               'goal': self.application.navigation.goal if ok else None}))


class ApiNavigationPreviewHandler(tornado.web.RequestHandler):
    """Static A* preview; deliberately does not touch ROS or live navigation."""
    async def post(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '请先进入导航页面'}))
            return
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            values = tuple(float(body[key]) for key in
                           ('start_x', 'start_y', 'goal_x', 'goal_y'))
            if not all(math.isfinite(value) for value in values):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '预演坐标无效'}))
            return
        if not self.application.planning_lock.acquire(blocking=False):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '已有一个全局路径请求正在计算，请等待其结束',
                'preview': None,
            }))
            return
        try:
            ok, message, preview = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, self.application.navigation.preview_plan, *values)
        finally:
            self.application.planning_lock.release()
        self.write(json.dumps({
            'success': ok,
            'message': message,
            'preview': preview if ok else None,
        }))


class ApiNavigationCancelHandler(tornado.web.RequestHandler):
    def post(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '当前不在导航模式'}))
            return
        self.application.navigation.clear_navigation('导航已取消，请重新设置目标点')
        self.application.teleop_owner = None
        self.application.web_node.publish_teleop_stop()
        self.application.web_node.publish_teleop_enable(False)
        self.application.navigation.set_chassis_enabled(False)
        self.application.web_node.publish_navigation_cancel(True)
        # Empty Path also clears any downstream reference-path visualization.
        self.application.web_node.publish_global_path([])
        self.write(json.dumps({
            'success': True,
            'message': '导航已取消，目标和规划路径已清除',
        }))


class ApiNavigationBackendHandler(tornado.web.RequestHandler):
    """Select one actuator command owner while the physical gate is locked."""

    def get(self):
        chassis = self.application.web_node.chassis_status
        self.write(json.dumps({
            'backend': chassis.get('navigation_backend', 'nav2'),
            'chassis_enabled': bool(chassis.get('enabled')),
            'nav2_online': (
                time.monotonic() - self.application.web_node.nav2_status_t < 1.5),
            'nav2': dict(self.application.web_node.nav2_status),
        }))

    async def post(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '请先进入导航页面'}))
            return
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            backend = str(body['backend']).strip().lower()
            if backend not in ('scan', 'nav2'):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({
                'success': False, 'message': '后端只能是 scan 或 nav2'}))
            return

        status = self.application.web_node.chassis_status
        if status.get('enabled'):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '底盘启用期间不能切换导航后端；请先取消并锁定底盘',
            }))
            return
        if backend == 'nav2':
            nav2_age = time.monotonic() - self.application.web_node.nav2_status_t
            nav2_state = self.application.web_node.nav2_status.get('state')
            if nav2_age >= 1.5 or nav2_state in ('offline', 'error'):
                self.set_status(409)
                self.write(json.dumps({
                    'success': False,
                    'message': 'Nav2 服务未就绪，不能选为真实控制后端',
                    'nav2': self.application.web_node.nav2_status,
                }))
                return

        # Invalidate every old trajectory before handing command ownership to
        # another producer. The gate itself independently refuses armed switch.
        self.application.navigation.clear_navigation(
            '正在切换导航后端，旧目标已清除')
        self.application.teleop_owner = None
        self.application.web_node.publish_teleop_stop()
        self.application.web_node.publish_teleop_enable(False)
        self.application.web_node.publish_navigation_cancel(True)
        self.application.web_node.publish_global_path([])
        self.application.web_node.publish_chassis_enable(False)
        await tornado.gen.sleep(0.10)
        self.application.web_node.publish_navigation_backend(backend)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            await tornado.gen.sleep(0.05)
            status = self.application.web_node.chassis_status
            if (not status.get('enabled') and
                    status.get('navigation_backend') == backend):
                self.write(json.dumps({
                    'success': True,
                    'message': '已选择%s；底盘保持锁定，请重新启用后发送新目标' %
                               backend.upper(),
                    'backend': backend,
                }))
                return
        self.set_status(503)
        self.write(json.dumps({
            'success': False,
            'message': '安全门未确认后端切换，底盘保持锁定',
            'status': status,
        }))




class ApiNavigationAlignmentHandler(tornado.web.RequestHandler):
    """Auto-refine a rough map position against a short live LIO cloud burst."""

    def get(self):
        self.write(json.dumps(self.application.navigation.alignment_status()))

    async def post(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '请先进入导航页面'}))
            return
        self.application.teleop_owner = None
        self.application.web_node.publish_teleop_stop()
        self.application.web_node.publish_teleop_enable(False)
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            map_x = float(body['x'])
            map_y = float(body['y'])
            yaw_value = body.get('yaw_deg')
            map_yaw_deg = None if yaw_value in (None, '') else float(yaw_value)
            values = (map_x, map_y) if map_yaw_deg is None else (
                map_x, map_y, map_yaw_deg)
            if not all(math.isfinite(value) for value in values):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '地图位姿标定参数无效'}))
            return

        # Calibration needs SCAN's base-pose adapter but an uncalibrated static
        # map may only be present in identity mode.  Start that isolated graph
        # to obtain a fresh pose, then restart after saving the transform.
        planner_ok, planner_message = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.operations.ensure_navigation)
        if not planner_ok:
            self.write(json.dumps({'success': False, 'message': planner_message}))
            return
        deadline = time.time() + 6.0
        while time.time() < deadline:
            if self.application.navigation.pose_t and time.time() - self.application.navigation.pose_t < 2.5:
                break
            await tornado.gen.sleep(0.10)

        if (not self.application.navigation.pose_t or
                time.time() - self.application.navigation.pose_t >= 2.5):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '导航基座位姿未就绪，请检查 SCAN 位姿适配器',
            }))
            return
        with self.application.navigation.lock:
            odom_before_capture = dict(self.application.navigation.odom_pose or {})

        # Recalibration is always a stop operation and can never preserve or
        # resume an old trajectory.  Capture only while the robot is locked.
        self.application.navigation.set_chassis_enabled(False)
        self.application.web_node.publish_navigation_cancel(True)
        self.application.web_node.publish_global_path([])
        if not self.application.web_node.begin_registration_capture():
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '另一次自动配准正在进行',
            }))
            return
        self.application.navigation.invalidate_alignment('正在采集点云并自动精配准')
        deadline = time.time() + 3.0
        while (self.application.web_node.registration_capture_count() < 8 and
               time.time() < deadline):
            await tornado.gen.sleep(0.10)
        clouds = self.application.web_node.finish_registration_capture()
        if len(clouds) < 4:
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '实时点云帧数不足，未执行标定',
                'captured_frames': len(clouds),
            }))
            return
        with self.application.navigation.lock:
            odom_pose = dict(self.application.navigation.odom_pose or {})
            static_points = self.application.navigation.static_points.copy()
        if not odom_pose:
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '导航里程计离线'}))
            return
        if (not odom_before_capture or
                math.hypot(odom_pose['x'] - odom_before_capture['x'],
                           odom_pose['y'] - odom_before_capture['y']) > 0.05 or
                abs(math.atan2(
                    math.sin(odom_pose['yaw'] - odom_before_capture['yaw']),
                    math.cos(odom_pose['yaw'] - odom_before_capture['yaw']))) >
                math.radians(2.0)):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '采集期间检测到机器人位姿变化，已拒绝标定',
            }))
            return
        rough_yaw = None if map_yaw_deg is None else math.radians(map_yaw_deg)
        def register():
            matcher = AutoMapRegistration(static_points)
            return matcher.register(
                clouds, odom_pose, map_x, map_y, rough_yaw)
        try:
            ok, message, registration = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, register)
        except Exception as exc:
            self.set_status(500)
            self.write(json.dumps({
                'success': False, 'message': '自动配准计算异常: %s' % exc,
            }))
            return
        if not ok:
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': message,
                'registration': registration,
            }))
            return
        pose_ok, pose_message = self.application.navigation.validate_alignment_pose(
            registration['x'], registration['y'])
        if not pose_ok:
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': pose_message,
                'registration': registration,
            }))
            return
        quality = registration['quality']
        rough_pose = {'x': map_x, 'y': map_y}
        if map_yaw_deg is not None:
            rough_pose['yaw_deg'] = map_yaw_deg
        ok, save_message = self.application.navigation.set_alignment(
            registration['x'], registration['y'], registration['yaw'],
            method='auto_icp', quality=quality, rough_pose=rough_pose)
        if not ok:
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': save_message}))
            return

        stop_ok, stop_message = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.operations.stop_planner)
        if not stop_ok:
            self.set_status(503)
            self.write(json.dumps({
                'success': False,
                'message': save_message + '；但 SCAN 停止异常: ' + stop_message,
                'alignment': self.application.navigation.alignment_status(),
            }))
            return
        planner_ok, planner_message = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.operations.ensure_navigation)
        result_summary = ('精配准位姿 (%.2f, %.2f, %.1f°)，30 cm 重合率 %.1f%%，'
                          'RMSE %.3f m' % (
                              registration['x'], registration['y'],
                              registration['yaw_deg'],
                              quality['inlier_ratio_30cm'] * 100.0,
                              quality['rmse_m']))
        response_message = (message + '；' + result_summary +
                            '；SCAN 已按新基准重启，底盘仍锁定'
                            if planner_ok else
                            message + '；' + result_summary +
                            '；SCAN 重启失败: ' + planner_message)
        if not planner_ok:
            self.set_status(503)
        self.write(json.dumps({
            'success': planner_ok,
            'message': response_message,
            'alignment': self.application.navigation.alignment_status(),
            'registration': registration,
        }))


def chassis_cancel_is_safe(status):
    """Return true only for an explicitly cancelled, disarmed, zero output."""
    if not isinstance(status, dict):
        return False
    if not status.get('cancelled') or status.get('enabled'):
        return False
    output = status.get('output')
    if not isinstance(output, (list, tuple)) or len(output) < 3:
        return False
    try:
        return max(abs(float(value)) for value in output[:3]) <= 1e-3
    except (TypeError, ValueError):
        return False


def chassis_cancel_acknowledged(status, status_age, previous_cancel_seq):
    """Require a fresh acknowledgement for the cancellation just published."""
    try:
        return (
            float(status_age) <= 1.0 and
            int(status.get('cancel_seq', 0) or 0) > int(previous_cancel_seq) and
            chassis_cancel_is_safe(status))
    except (AttributeError, TypeError, ValueError):
        return False


class ApiNavigationChassisHandler(tornado.web.RequestHandler):
    """Explicitly arm or lock the physical chassis safety gate."""

    def get(self):
        self.write(json.dumps(self.application.web_node.chassis_status))

    async def post(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '请先进入导航页面'}))
            return
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            enabled = body['enabled']
            if not isinstance(enabled, bool):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '底盘开关参数无效'}))
            return
        if enabled:
            if self.application.web_node.chassis_status.get('teleop_enabled'):
                self.set_status(409)
                self.write(json.dumps({
                    'success': False,
                    'message': '键盘控制已开启；请使用键盘控制开关关闭并锁定底盘',
                }))
                return
            if not self.application.navigation.is_alignment_valid():
                self.set_status(409)
                self.write(json.dumps({
                    'success': False,
                    'message': '请先完成本次开机后的地图位姿标定',
                }))
                return
            planner_ok, planner_message = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, self.application.operations.ensure_navigation)
            if not planner_ok:
                self.write(json.dumps({
                    'success': False, 'message': planner_message,
                }))
                return
            # Give the safety gate a short bounded interval to see SCAN odometry.
            deadline = time.time() + 2.0
            while time.time() < deadline:
                status = self.application.web_node.chassis_status
                if status.get('ready'):
                    break
                await tornado.gen.sleep(0.10)
        status = self.application.web_node.chassis_status
        if enabled and not status.get('ready'):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '底盘尚未就绪：' + str(status.get('reason') or '安全门离线'),
                'status': status,
            }))
            return
        if enabled:
            # Arming can never resume a previously published trajectory.  Force
            # SCAN and the UI back to a cancelled state; the operator must send
            # a fresh goal after the gate reports enabled.
            self.application.navigation.clear_navigation(
                '底盘已启用，请检查环境后发送新的目标点')
            previous_cancel_seq = int(status.get('cancel_seq', 0) or 0)
            # Publish exactly one reliable cancellation, then wait for the
            # gate to acknowledge that specific message. Accepting an older
            # safe state while still publishing a new cancel creates a
            # cross-topic race: the late cancel can otherwise arrive just
            # after the enable request and immediately lock the chassis again.
            cancel_confirmed = False
            self.application.web_node.publish_navigation_cancel(True)
            self.application.web_node.publish_global_path([])
            cancel_deadline = time.monotonic() + 3.0
            while not cancel_confirmed and time.monotonic() < cancel_deadline:
                await tornado.gen.sleep(0.05)
                status = self.application.web_node.chassis_status
                status_age = time.monotonic() - getattr(
                    self.application.web_node, 'chassis_status_t', 0.0)
                cancel_confirmed = chassis_cancel_acknowledged(
                    status, status_age, previous_cancel_seq)
            if not cancel_confirmed:
                self.set_status(503)
                self.write(json.dumps({
                    'success': False,
                    'message': '底盘安全门未确认旧路线已取消，保持锁定',
                }))
                return
        ok, message = self.application.operations.set_chassis_enabled(enabled)
        if not enabled:
            self.application.teleop_owner = None
            self.application.web_node.publish_teleop_stop()
            self.application.web_node.publish_teleop_enable(False)
            self.application.navigation.clear_navigation('底盘已锁定，请重新设置目标点')
            self.application.web_node.publish_navigation_cancel(True)
            self.application.web_node.publish_global_path([])
        if enabled and ok:
            arm_deadline = time.monotonic() + 3.0
            while time.monotonic() < arm_deadline:
                status = self.application.web_node.chassis_status
                if status.get('enabled'):
                    break
                await tornado.gen.sleep(0.05)
            else:
                ok = False
                message = '底盘安全门拒绝启用：' + str(
                    status.get('reason') or '状态确认超时')
        self.write(json.dumps({'success': ok, 'message': message}))


class ApiNavigationTeleopHandler(tornado.web.RequestHandler):
    """Select/arm the exclusive keyboard input, or stop and lock it."""

    def get(self):
        status = self.application.web_node.chassis_status
        self.write(json.dumps({
            'enabled': bool(status.get('teleop_enabled')),
            'chassis_enabled': bool(status.get('enabled')),
            'control_mode': status.get('control_mode', 'navigation'),
            'limits': [TELEOP_VX, TELEOP_VY, TELEOP_VYAW],
        }))

    def _disable(self, message='键盘控制已关闭，底盘已锁定'):
        self.application.teleop_owner = None
        self.application.web_node.publish_teleop_stop()
        self.application.web_node.publish_teleop_enable(False)
        self.application.navigation.set_chassis_enabled(False)
        self.application.web_node.publish_navigation_cancel(True)
        self.application.web_node.publish_global_path([])
        self.application.navigation.clear_navigation(message)
        if self.application.operations.mode == 'navigation':
            self.application.camera.stop()

    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            enabled = body['enabled']
            client_id = str(body.get('client_id', ''))
            if not isinstance(enabled, bool):
                raise ValueError()
            if enabled and (len(client_id) != 32 or
                            any(value not in '0123456789abcdef' for value in client_id)):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '键盘控制参数无效'}))
            return

        if not enabled:
            self._disable()
            self.write(json.dumps({'success': True,
                                   'message': '键盘控制已关闭，底盘已锁定'}))
            return

        status = self.application.web_node.chassis_status
        owner = self.application.teleop_owner
        if owner and owner != client_id and status.get('teleop_enabled'):
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '另一个网页正在使用键盘控制',
            }))
            return
        # Cancel and positively observe the old route being rejected before
        # selecting the new command source.
        self.application.navigation.clear_navigation('正在切换到键盘控制')
        previous_cancel_seq = int(status.get('cancel_seq', 0) or 0)
        cancel_confirmed = False
        self.application.web_node.publish_navigation_cancel(True)
        self.application.web_node.publish_global_path([])
        cancel_deadline = time.monotonic() + 3.0
        while not cancel_confirmed and time.monotonic() < cancel_deadline:
            await tornado.gen.sleep(0.05)
            status = self.application.web_node.chassis_status
            status_age = time.monotonic() - getattr(
                self.application.web_node, 'chassis_status_t', 0.0)
            cancel_confirmed = chassis_cancel_acknowledged(
                status, status_age, previous_cancel_seq)
        if not cancel_confirmed:
            self._disable('键盘控制切换失败，底盘保持锁定')
            self.set_status(503)
            self.write(json.dumps({
                'success': False, 'message': '底盘安全门未确认旧导航已取消',
            }))
            return

        self.application.teleop_owner = client_id
        self.application.web_node.publish_teleop_enable(True)
        select_deadline = time.monotonic() + 3.0
        while time.monotonic() < select_deadline:
            status = self.application.web_node.chassis_status
            status_age = time.monotonic() - getattr(
                self.application.web_node, 'chassis_status_t', 0.0)
            if (status_age <= 1.0 and status.get('teleop_enabled') and
                    status.get('ready')):
                break
            await tornado.gen.sleep(0.05)
        else:
            reason = str(status.get('reason') or '安全门未就绪')
            self._disable('键盘控制未能启用，底盘保持锁定')
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '键盘控制未就绪：' + reason,
            }))
            return

        # Publish a fresh zero command on the selected input before and after
        # arming, so enabling the switch can never reuse an old nonzero sample.
        self.application.web_node.publish_teleop_stop()
        ok, message = self.application.navigation.set_chassis_enabled(
            True, require_alignment=False)
        if not ok:
            self._disable('键盘控制未能启用，底盘保持锁定')
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': message}))
            return
        arm_deadline = time.monotonic() + 3.0
        while time.monotonic() < arm_deadline:
            self.application.web_node.publish_teleop_stop()
            status = self.application.web_node.chassis_status
            status_age = time.monotonic() - getattr(
                self.application.web_node, 'chassis_status_t', 0.0)
            if (status_age <= 1.0 and status.get('enabled') and
                    status.get('teleop_enabled')):
                break
            await tornado.gen.sleep(0.05)
        else:
            reason = str(status.get('reason') or '状态确认超时')
            self._disable('键盘控制未能启用，底盘保持锁定')
            self.set_status(503)
            self.write(json.dumps({
                'success': False, 'message': '底盘拒绝键盘控制：' + reason,
            }))
            return

        camera_ok, camera_message = True, ''
        if self.application.operations.mode == 'navigation':
            camera_ok, camera_message = self.application.camera.start()
        self.application.navigation.clear_navigation(
            '键盘控制已启用：W/S 前后，A/D 侧移，Q/E 转向')
        suffix = '' if camera_ok else '；' + camera_message
        self.write(json.dumps({
            'success': True,
            'message': '键盘控制已启用，松开按键即停车' + suffix,
            'limits': [TELEOP_VX, TELEOP_VY, TELEOP_VYAW],
        }))


class ApiNavigationPostureHandler(tornado.web.RequestHandler):
    """Stop motion, then request StandDown or guarded StandUp->BalanceStand."""

    ACTION_LABELS = {
        'stand_down': '卧倒',
        'recovery_stand': '两阶段恢复可移动姿态',
    }

    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            action = str(body.get('action', ''))
            client_id = str(body.get('client_id', ''))
            if action not in self.ACTION_LABELS:
                raise ValueError()
            if (len(client_id) != 32 or
                    any(value not in '0123456789abcdef' for value in client_id)):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '姿态动作参数无效'}))
            return

        status = self.application.web_node.chassis_status
        if not status.get('connected'):
            self.set_status(503)
            self.write(json.dumps({
                'success': False, 'message': 'Sport API 未连接，不能执行姿态动作',
            }))
            return
        if status.get('posture_busy') or status.get('posture_pending'):
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '已有姿态动作正在执行，请稍候',
            }))
            return
        owner = self.application.teleop_owner
        if status.get('teleop_enabled') and owner != client_id:
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '另一个网页正在使用键盘控制',
            }))
            return

        # Stop both command sources and invalidate any existing route before
        # the safety gate accepts the one-shot posture command.
        previous_seq = int(status.get('posture_seq', 0) or 0)
        self.application.teleop_owner = None
        self.application.web_node.publish_teleop_stop()
        self.application.web_node.publish_teleop_enable(False)
        self.application.navigation.set_chassis_enabled(False)
        self.application.web_node.publish_navigation_cancel(True)
        self.application.web_node.publish_global_path([])
        self.application.navigation.clear_navigation(
            '姿态动作已请求，底盘保持锁定')
        self.application.web_node.publish_posture_command(action)

        deadline = time.time() + 1.0
        while time.time() < deadline:
            status = self.application.web_node.chassis_status
            if int(status.get('posture_seq', 0) or 0) > previous_seq:
                break
            await tornado.gen.sleep(0.05)
        else:
            self.set_status(503)
            self.write(json.dumps({
                'success': False,
                'message': '底盘安全门未确认姿态动作，底盘已保持锁定',
            }))
            return

        if action == 'stand_down':
            message = '卧倒动作已发送，底盘保持锁定'
        else:
            message = '两阶段恢复已开始：先站立锁定，再恢复可移动状态'
        self.write(json.dumps({
            'success': True,
            'message': message,
            'posture_state': status.get('posture_state'),
        }))


class ApiNavigationCameraHandler(tornado.web.RequestHandler):
    def get(self):
        if self.application.operations.mode != 'navigation':
            self.set_status(409)
            return
        image = self.application.camera.image()
        if image is None:
            self.set_status(503)
            return
        self.set_header('Content-Type', 'image/jpeg')
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.write(image)


class ApiRemembrStatusHandler(tornado.web.RequestHandler):
    def get(self):
        self.set_header('Content-Type', 'application/json; charset=utf-8')
        self.set_header('Cache-Control', 'no-store')
        self.write(json.dumps(self.application.remembr.status()))


class ApiRemembrControlHandler(tornado.web.RequestHandler):
    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            enabled = body['enabled']
            if not isinstance(enabled, bool):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({
                'success': False, 'message': 'enabled 必须是布尔值',
            }))
            return
        if enabled and self.application.operations.mode != 'navigation':
            self.set_status(409)
            self.write(json.dumps({
                'success': False, 'message': '请先进入导航模式',
            }))
            return
        ok, message = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.remembr.set_enabled, enabled)
        if not ok:
            self.set_status(409)
        self.write(json.dumps({
            'success': ok, 'message': message,
            'status': self.application.remembr.status(),
        }))


class ApiRemembrMemoryHandler(tornado.web.RequestHandler):
    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            caption = str(body.get('caption', '')).strip()
            if not caption or len(caption) > 3000:
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({
                'success': False, 'message': '记忆描述不能为空且不能超过 3000 字符',
            }))
            return
        ok, message, memory_id = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.remembr.add_manual, caption)
        if not ok:
            self.set_status(409)
        self.write(json.dumps({
            'success': ok, 'message': message, 'memory_id': memory_id,
        }))


class ApiRemembrQueryHandler(tornado.web.RequestHandler):
    """Retrieve and preview one candidate; this endpoint never commands motion."""

    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            if not isinstance(body, dict):
                raise ValueError()
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': 'JSON 格式错误'}))
            return
        if not self.application.planning_lock.acquire(blocking=False):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '已有一个全局路径请求正在计算，请等待其结束',
                'memories': [], 'candidate': None,
            }))
            return
        try:
            result = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, self.application.remembr.query, body)
        finally:
            self.application.planning_lock.release()
        if not result.get('success'):
            self.set_status(400)
        self.write(json.dumps(result))


class ApiSaveHandler(tornado.web.RequestHandler):
    """POST /api/save — 保存当前建图结果"""
    async def post(self):
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            name = str(body.get('name', '')).strip()
        except Exception:
            name = ''
        if not name:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '请输入地图名'}))
            return
        ok, msg = await tornado.ioloop.IOLoop.current().run_in_executor(
            None, self.application.web_node.do_save, name)
        self.write(json.dumps({'success': ok, 'message': msg}))


class ApiMapsHandler(tornado.web.RequestHandler):
    def get(self):
        files = []
        active_name = navigation_source_map_name()
        if os.path.isdir(MAPS_DIR):
            for fn in sorted(os.listdir(MAPS_DIR)):
                fp = os.path.join(MAPS_DIR, fn)
                if os.path.isfile(fp) and valid_map_name(fn):
                    files.append({
                        'name': fn,
                        'active_navigation': fn == active_name,
                        'protected': fn in (NAVIGATION_SOURCE_MAP, active_name),
                    })
        self.write(json.dumps({'maps': files}))

    async def post(self):
        files = self.request.files.get('file', [])
        if len(files) != 1:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '请选择一个 .npz 地图文件'}))
            return
        upload = files[0]
        name = os.path.basename(upload.get('filename', '')).strip()
        body = upload.get('body', b'')
        path = map_file_path(name)
        if path is None or not body:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '仅支持有效的 .npz 地图文件'}))
            return
        if len(body) > MAX_UPLOAD_BYTES:
            self.set_status(413)
            self.write(json.dumps({'success': False, 'message': '地图文件不能超过 100 MB'}))
            return
        if os.path.exists(path):
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '同名地图已经存在'}))
            return
        os.makedirs(MAPS_DIR, exist_ok=True)
        temporary = os.path.join(MAPS_DIR, '.upload-%s.npz' % uuid.uuid4().hex)
        try:
            with open(temporary, 'wb') as stream:
                stream.write(body)
            await tornado.ioloop.IOLoop.current().run_in_executor(
                None, load_saved_map, temporary)
            os.replace(temporary, path)
            self.write(json.dumps({'success': True, 'message': '已添加地图 ' + name,
                                   'name': name}))
        except Exception as exc:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '地图校验失败: %s' % exc}))


class ApiMapPreviewHandler(tornado.web.RequestHandler):
    async def get(self):
        name = self.get_argument('name', '')
        path = map_file_path(name)
        if path is None or not os.path.isfile(path):
            self.set_status(404)
            self.write(json.dumps({'success': False, 'message': '地图不存在'}))
            return
        try:
            preview = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, build_saved_map_preview, path)
            preview.update({'success': True, 'name': name})
            self.set_header('Cache-Control', 'no-store')
            self.write(json.dumps(preview, separators=(',', ':')))
        except Exception as exc:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '预览失败: %s' % exc}))


class ApiMapEraseHandler(tornado.web.RequestHandler):
    """Non-destructive multi-polygon erase with hard floor/roof guards."""
    async def post(self):
        if self.application.operations.mode != 'maps':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '请先进入地图管理页面'}))
            return
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            source_name = str(body['name']).strip()
            output_name = str(body['output_name']).strip()
            if not output_name.lower().endswith('.npz'):
                output_name += '.npz'
            raw_regions = body.get('regions')
            if raw_regions is None:
                # Accept the original single-box contract for old open pages.
                raw_regions = [body]
            regions = raw_regions
        except Exception:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '擦除参数无效'}))
            return
        source_path = map_file_path(source_name)
        output_path = map_file_path(output_name)
        if source_path is None or not os.path.isfile(source_path):
            self.set_status(404)
            self.write(json.dumps({'success': False, 'message': '原地图不存在'}))
            return
        if output_path is None:
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '新地图名无效'}))
            return
        if output_path == source_path:
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '必须另存为新地图，不能覆盖原图'}))
            return
        if os.path.exists(output_path):
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '新地图名已经存在'}))
            return

        active_source = navigation_source_map_name()
        rebuild_navigation = source_name == active_source
        backup_dir = None
        try:
            stats = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, erase_saved_map_regions, source_path, output_path, regions)
            if rebuild_navigation:
                metadata, backup_dir = await tornado.ioloop.IOLoop.current().run_in_executor(
                    None, switch_navigation_baseline,
                    self.application.navigation, output_path)
                self.application.navigation_map_json = json.dumps(
                    self.application.navigation.static_map_payload(), separators=(',', ':'))
                stats['navigation_occupied_cells'] = metadata.get('occupied_cells')
                stats['navigation_reachable_cells'] = metadata.get(
                    'free_connectivity', {}).get('inflated', {}).get(
                        'start_component_cells')
            message = '已另存 %s，合并 %d 个不规则区域擦除 %d 点；地面和 2.20 m 以上顶棚点均保留' % (
                output_name, stats['region_count'], stats['removed_points'])
            if rebuild_navigation:
                message += '；导航地图已重建并切换'
            self.write(json.dumps({
                'success': True,
                'message': message,
                'name': output_name,
                'stats': stats,
                'navigation_rebuilt': rebuild_navigation,
                'navigation_backup': backup_dir,
            }))
        except Exception as exc:
            # The output is new and disposable; leave the original untouched.
            if backup_dir:
                try:
                    await tornado.ioloop.IOLoop.current().run_in_executor(
                        None, restore_navigation_backup, backup_dir)
                    await tornado.ioloop.IOLoop.current().run_in_executor(
                        None, self.application.navigation.reload_map)
                    self.application.navigation_map_json = json.dumps(
                        self.application.navigation.static_map_payload(), separators=(',', ':'))
                except Exception:
                    pass
            try:
                os.unlink(output_path)
            except OSError:
                pass
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '擦除失败: %s' % exc}))


class ApiMapNavigationBaselineHandler(tornado.web.RequestHandler):
    """Select a saved NPZ and atomically make it the navigation baseline."""
    async def post(self):
        if self.application.operations.mode != 'maps':
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': '请先进入地图管理页面'}))
            return
        if self.application.operations.planner_active(refresh=True):
            self.set_status(409)
            self.write(json.dumps({'success': False, 'message': 'SCAN-Planner 仍在运行，请先停止导航'}))
            return
        try:
            body = json.loads(self.request.body.decode('utf-8') or '{}')
            name = str(body['name']).strip()
        except Exception:
            name = ''
        path = map_file_path(name)
        if path is None or not os.path.isfile(path):
            self.set_status(404)
            self.write(json.dumps({'success': False, 'message': '地图不存在'}))
            return
        if name == navigation_source_map_name():
            self.write(json.dumps({
                'success': True, 'message': name + ' 已经是导航基准', 'name': name,
            }))
            return
        backup_dir = None
        try:
            # Validate early so malformed uploads fail before a costly build.
            await tornado.ioloop.IOLoop.current().run_in_executor(
                None, load_saved_map, path)
            metadata, backup_dir = await tornado.ioloop.IOLoop.current().run_in_executor(
                None, switch_navigation_baseline, self.application.navigation, path)
            self.application.navigation.clear_navigation('导航基准已切换，请重新设置目标点')
            self.application.navigation_map_json = json.dumps(
                self.application.navigation.static_map_payload(), separators=(',', ':'))
            connectivity = metadata.get('free_connectivity', {}).get('inflated', {})
            self.write(json.dumps({
                'success': True,
                'message': '已将 %s 设为导航基准，导航地图已重建' % name,
                'name': name,
                'point_count': int(metadata.get('source_points', 0)),
                'occupied_cells': int(metadata.get('occupied_cells', 0)),
                'reachable_cells': int(connectivity.get('start_component_cells', 0)),
                'backup': backup_dir,
            }))
        except Exception as exc:
            if backup_dir:
                try:
                    await tornado.ioloop.IOLoop.current().run_in_executor(
                        None, restore_navigation_backup, backup_dir)
                    await tornado.ioloop.IOLoop.current().run_in_executor(
                        None, self.application.navigation.reload_map)
                    self.application.navigation_map_json = json.dumps(
                        self.application.navigation.static_map_payload(),
                        separators=(',', ':'))
                except Exception:
                    pass
            self.set_status(400)
            self.write(json.dumps({'success': False, 'message': '切换导航基准失败: %s' % exc}))


class ApiMapDeleteHandler(tornado.web.RequestHandler):
    async def post(self):
        try:
            name = str(json.loads(self.request.body.decode('utf-8') or '{}')['name'])
        except Exception:
            name = ''
        path = map_file_path(name)
        if path is None or not os.path.isfile(path):
            self.set_status(404)
            self.write(json.dumps({'success': False, 'message': '地图不存在'}))
            return
        if name in (NAVIGATION_SOURCE_MAP, navigation_source_map_name()):
            self.set_status(409)
            self.write(json.dumps({
                'success': False,
                'message': '该地图是当前导航基准图，不能删除',
            }))
            return
        try:
            os.unlink(path)
            self.write(json.dumps({'success': True, 'message': '已删除地图 ' + name}))
        except OSError as exc:
            self.set_status(500)
            self.write(json.dumps({'success': False, 'message': '删除失败: %s' % exc}))


class ApiDownloadHandler(tornado.web.RequestHandler):
    def get(self):
        name = self.get_argument('name', '')
        if not name or '/' in name or '..' in name:
            self.set_status(400)
            self.write('bad name')
            return
        fp = os.path.join(MAPS_DIR, name)
        if not os.path.isfile(fp):
            self.set_status(404)
            self.write('not found')
            return
        self.set_header('Content-Type', 'application/octet-stream')
        self.set_header('Content-Disposition', 'attachment; filename="%s"' % name)
        with open(fp, 'rb') as f:
            self.write(f.read())


class WsHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        self.teleop_client_id = None
        self.application.ws_clients.add(self)

    def on_message(self, message):
        try:
            data = json.loads(message)
            if not isinstance(data, dict) or data.get('type') != 'teleop':
                return
            client_id = str(data.get('client_id', ''))
            if client_id != self.application.teleop_owner:
                return
            status = self.application.web_node.chassis_status
            if not status.get('teleop_enabled') or not status.get('enabled'):
                return
            axes = tuple(float(data[key]) for key in ('forward', 'lateral', 'turn'))
            if not all(math.isfinite(value) and abs(value) <= 1.0 for value in axes):
                raise ValueError()
            self.teleop_client_id = client_id
            self.application.web_node.publish_teleop_axes(*axes)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            # Invalid remote input is never forwarded.  The 350 ms chassis
            # watchdog then stops and locks if valid commands do not resume.
            return

    def on_close(self):
        self.application.ws_clients.discard(self)
        if (self.teleop_client_id and
                self.teleop_client_id == self.application.teleop_owner):
            self.application.teleop_owner = None
            self.application.web_node.publish_teleop_stop()
            self.application.web_node.publish_teleop_enable(False)
            self.application.navigation.set_chassis_enabled(False)
            self.application.web_node.publish_navigation_cancel(True)
            self.application.web_node.publish_global_path([])
            self.application.navigation.clear_navigation(
                '键盘控制页面已断开，底盘已自动锁定')


class Application(tornado.web.Application):

    def __init__(self, web_node, navigation, camera, operations, remembr):
        self.web_node = web_node
        self.navigation = navigation
        self.camera = camera
        self.operations = operations
        self.remembr = remembr
        self.teleop_owner = None
        # Global footprint A* is CPU-heavy and cannot be safely duplicated by
        # refreshing the page or opening a second browser tab.
        self.planning_lock = threading.Lock()
        self.navigation_map_json = json.dumps(navigation.static_map_payload(),
                                              separators=(',', ':'))
        self.ws_clients = set()
        static_dir = os.path.join(os.path.dirname(__file__), 'static')
        handlers = [
            (r'/', IndexHandler),
            (r'/static/(.*)', tornado.web.StaticFileHandler, {'path': static_dir}),
            (r'/api/status', ApiStatusHandler),
            (r'/api/mode', ApiModeHandler),
            (r'/api/mapping', ApiMappingHandler),
            (r'/api/save', ApiSaveHandler),
            (r'/api/maps', ApiMapsHandler),
            (r'/api/maps/preview', ApiMapPreviewHandler),
            (r'/api/maps/erase', ApiMapEraseHandler),
            (r'/api/maps/navigation-baseline', ApiMapNavigationBaselineHandler),
            (r'/api/maps/delete', ApiMapDeleteHandler),
            (r'/api/download', ApiDownloadHandler),
            (r'/api/navigation/map3d', ApiNavigationMapHandler),
            (r'/api/navigation/goal', ApiNavigationGoalHandler),
            (r'/api/navigation/preview', ApiNavigationPreviewHandler),
            (r'/api/navigation/cancel', ApiNavigationCancelHandler),
            (r'/api/navigation/backend', ApiNavigationBackendHandler),
            (r'/api/navigation/alignment', ApiNavigationAlignmentHandler),
            (r'/api/navigation/chassis', ApiNavigationChassisHandler),
            (r'/api/navigation/teleop', ApiNavigationTeleopHandler),
            (r'/api/navigation/posture', ApiNavigationPostureHandler),
            (r'/api/navigation/camera.jpg', ApiNavigationCameraHandler),
            (r'/api/remembr/status', ApiRemembrStatusHandler),
            (r'/api/remembr/control', ApiRemembrControlHandler),
            (r'/api/remembr/memory', ApiRemembrMemoryHandler),
            (r'/api/remembr/query', ApiRemembrQueryHandler),
            (r'/ws', WsHandler),
        ]
        super().__init__(handlers)


def push_frame(app, payload):
    for ws in list(app.ws_clients):
        try:
            ws.write_message(payload)
        except Exception:
            app.ws_clients.discard(ws)


def broadcast_loop(app, ioloop):
    """5Hz 推送当前模式；重型地图/雷达数据永不跨页面发送。"""
    n = 0
    while True:
        n += 1
        try:
            mode = app.operations.mode
            if mode == 'mapping':
                map_tick = n % int(BROADCAST_HZ / MAP_PUSH_HZ) == 0
                snap = app.web_node.snapshot(with3d=map_tick)
                frame = {
                    'mode': 'mapping', 't': time.time(),
                    'map_png': snap['map_png'] if map_tick else None,
                    'map3d': snap['map3d'] if map_tick else [],
                    'ground3d': [],
                    'map': snap['map'], 'pose': snap['pose'],
                    'lidar': snap['lidar'], 'lidar3d': snap['lidar3d'],
                    'lidar_rate': snap['lidar_rate'],
                    'map_stats': snap['map_stats'], 'health': snap['health'],
                }
            elif mode == 'navigation':
                frame = app.navigation.snapshot(
                    app.camera.status(), app.operations.planner_active(),
                    chassis_status=app.web_node.chassis_status,
                    with_paths=(n % 2 == 0),
                    with_obstacles=(n % 2 == 0))
                frame['t'] = time.time()
            else:
                frame = {'mode': 'maps', 't': time.time()}
            # The top navigation controls are global.  Keep their safety state
            # live without enabling any heavy map, camera or SCAN stream.
            frame['chassis'] = dict(app.web_node.chassis_status)
            frame['nav2'] = dict(app.web_node.nav2_status)
            frame['nav2']['online'] = (
                time.monotonic() - app.web_node.nav2_status_t < 1.5)
            frame['nav2_shadow'] = dict(app.web_node.nav2_shadow_metrics)
            payload = json.dumps(frame)
            ioloop.add_callback(push_frame, app, payload)
        except Exception:
            pass
        time.sleep(1.0 / BROADCAST_HZ)


def main(args=None):
    rclpy.init(args=args)
    navigation = NavigationState()
    camera = CameraBridge()
    node = WebNode(navigation)

    # 在 ROS executor 开始前就根据持久运行的 SCAN 服务确定首屏模式，
    # 避免导航首屏短暂订阅建图大点云。
    operations = OperationManager(
        navigation, camera, node.do_save, node.set_mapping_streams,
        node.reset_mapping)
    try:
        remembr = RemembrService(camera, navigation)
    except Exception as exc:
        # Semantic memory is optional. A malformed model config or damaged DB
        # must not prevent the established SLAM/navigation console from booting.
        remembr = UnavailableRemembrService(exc)
        print('端侧语义记忆已安全降级: %s' % exc, flush=True)

    # ROS 回调已在独立线程，且预览点云已由 C++ 抽稀；单线程执行器避免默认
    # MultiThreadedExecutor 在本机 CycloneDDS 下空转占用多个 CPU 核。
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ros_thread = threading.Thread(target=ex.spin, daemon=True)
    ros_thread.start()

    app = Application(node, navigation, camera, operations, remembr)
    app.listen(PORT, address='0.0.0.0')
    print('GO2-W 建图/导航控制台: http://<机器人IP>:%d' % PORT, flush=True)

    ioloop = tornado.ioloop.IOLoop.current()
    threading.Thread(target=broadcast_loop, args=(app, ioloop), daemon=True).start()

    try:
        ioloop.start()
    except KeyboardInterrupt:
        pass
    remembr.shutdown()
    operations.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
