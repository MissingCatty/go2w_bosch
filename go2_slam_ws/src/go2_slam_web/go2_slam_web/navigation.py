"""Navigation data, static-map routing and mode lifecycle for the GO2-W Web UI."""

import heapq
import hashlib
import json
import math
import os
import subprocess
import threading
import time
import uuid

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse import csgraph
from scipy.spatial import cKDTree

try:
    import cv2
except ImportError:  # The camera remains usable if OpenCV is absent.
    cv2 = None


NAV_DIR = '/home/unitree/go2_slam_ws/maps/navigation'
NAV_STEM = 'map_20260811_155640_273_nav'
NAV_META = os.path.join(NAV_DIR, NAV_STEM + '.json')
NAV_GRID = os.path.join(NAV_DIR, NAV_STEM + '_inflated.pgm')
NAV_ALIGNMENT = os.path.join(NAV_DIR, NAV_STEM + '_alignment.json')
CAMERA_EXE = ('/home/unitree/go2_slam_ws/install/go2_imu_bridge/lib/'
              'go2_imu_bridge/front_camera_bridge')
CAMERA_FILE = '/tmp/go2_front_camera_web.jpg'
SCAN_START = '/home/unitree/scan_planner_ws/start_go2w_dry.sh'
SCAN_STOP = '/home/unitree/scan_planner_ws/stop_go2w.sh'
LIO_NAV_RESTORE = '/home/unitree/go2_slam_ws/restore_lio_sam_navigation.sh'

# SCAN checks a pair of inflated cylinders rather than a single circular
# footprint.  The PGM is inflated by the matching 0.23 m cylinder radius; Web
# routing additionally checks the front/rear centres at +/-0.12 m.  Sixteen
# motion headings keep that directional check useful in narrow halls without
# allowing instantaneous right-angle corners.
FOOTPRINT_OFFSET_M = 0.12
FOOTPRINT_DIRECTIONS = (
    (1, 0), (2, 1), (1, 1), (1, 2),
    (0, 1), (-1, 2), (-1, 1), (-2, 1),
    (-1, 0), (-2, -1), (-1, -1), (-1, -2),
    (0, -1), (1, -2), (1, -1), (2, -1),
)
FOOTPRINT_ASTAR_TIMEOUT = 20.0
FOOTPRINT_ASTAR_YIELD_INTERVAL = 2048
# Weighted A* keeps the complete full-map search but prioritizes progress over
# exhaustive proof of the exact shortest path. Collision validity is unchanged;
# 1.25 caps the intended path-cost tradeoff while avoiding multi-second plateaus.
FOOTPRINT_HEURISTIC_WEIGHT = 1.25
# The saved grid already contains the 0.23 m hard body-radius inflation.  This
# additional band is deliberately a soft cost: open corridors strongly prefer
# their centre, while a narrow but collision-free passage remains traversable.
# This distance is measured outside the already radius-inflated hard grid.
GLOBAL_CLEARANCE_SOFT_M = 0.80
GLOBAL_CLEARANCE_WEIGHT = 4.00
# If map drift puts the physical robot inside a saved static obstacle, do not
# falsify the static map or let global A* start inside a wall. Pick a nearby
# entry point that is free in both the saved grid and the current live layer;
# SCAN then reaches that entry using live lidar only.
TEMP_START_MIN_RADIUS_M = 0.30
TEMP_START_MAX_RADIUS_M = 2.00
TEMP_START_OPEN_CLEARANCE_M = 0.20
TEMP_START_LIVE_MAX_AGE_S = 1.00


class FootprintSearchTimeout(RuntimeError):
    """Raised when an unreachable footprint search would monopolize Web."""


def _boot_id():
    try:
        with open('/proc/sys/kernel/random/boot_id', 'r', encoding='ascii') as stream:
            return stream.read().strip()
    except OSError:
        return ''


def _map_signature(meta):
    """Stable identity for the currently promoted navigation artifacts."""
    relevant = dict(meta)
    relevant['source'] = os.path.realpath(str(meta.get('source', '')))
    encoded = json.dumps(
        relevant, sort_keys=True, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _read_pgm(path):
    """Read the binary P5 maps emitted by the navigation-map converter."""
    with open(path, 'rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('only binary P5 PGM is supported')

        def token():
            while True:
                line = stream.readline()
                if not line:
                    raise ValueError('truncated PGM header')
                line = line.split(b'#', 1)[0]
                values = line.split()
                if values:
                    return values

        size = token()
        if len(size) == 1:
            size += token()
        width, height = int(size[0]), int(size[1])
        maximum = int(token()[0])
        if maximum > 255:
            raise ValueError('16-bit PGM is unsupported')
        pixels = np.frombuffer(stream.read(width * height), dtype=np.uint8)
        if pixels.size != width * height:
            raise ValueError('truncated PGM pixels')
        return pixels.reshape(height, width)


def _flat(points, digits=3):
    values = []
    append = values.append
    for point in points:
        append(round(float(point[0]), digits))
        append(round(float(point[1]), digits))
        append(round(float(point[2]), digits))
    return values


class CameraBridge:
    """Run Unitree VideoClient only while navigation is the active Web mode."""

    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.started_at = 0.0

    def start(self):
        with self.lock:
            if self.process and self.process.poll() is None:
                return True, '摄像头已启动'
            if not os.path.isfile(CAMERA_EXE):
                return False, '前置摄像头桥未编译'
            try:
                if os.path.exists(CAMERA_FILE):
                    os.unlink(CAMERA_FILE)
                self.process = subprocess.Popen(
                    [CAMERA_EXE, 'eth0', CAMERA_FILE, '4'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.started_at = time.time()
            except Exception as exc:
                self.process = None
                return False, '摄像头启动失败: %s' % exc
        time.sleep(0.15)
        with self.lock:
            if self.process and self.process.poll() is not None:
                self.process = None
                return False, '摄像头桥启动后异常退出'
        return True, '摄像头已启动'

    def stop(self):
        with self.lock:
            process = self.process
            self.process = None
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        for path in (CAMERA_FILE, CAMERA_FILE + '.part'):
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass

    def image(self):
        sample = self.sample(include_fingerprint=False)
        return sample['data'] if sample is not None else None

    @staticmethod
    def _fingerprint(data):
        """Return a compact perceptual dHash without retaining image pixels."""
        if cv2 is None:
            return None
        try:
            encoded = np.frombuffer(data, dtype=np.uint8)
            gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                return None
            thumb = cv2.resize(gray, (9, 8), interpolation=cv2.INTER_AREA)
            differences = (thumb[:, 1:] > thumb[:, :-1]).reshape(-1)
            fingerprint = 0
            for different in differences:
                fingerprint = (fingerprint << 1) | int(bool(different))
            return fingerprint
        except Exception:
            return None

    def sample(self, include_fingerprint=True):
        """Return one fresh atomic JPEG plus a token for frame deduplication."""
        try:
            with open(CAMERA_FILE, 'rb') as stream:
                stat = os.fstat(stream.fileno())
                age = time.time() - stat.st_mtime
                if age > 2.5:
                    return None
                data = stream.read()
            if len(data) > 4 and data[:2] == b'\xff\xd8':
                sample = {
                    'data': data,
                    'observed_at': stat.st_mtime,
                    'token': (stat.st_mtime_ns, stat.st_size),
                }
                if include_fingerprint:
                    sample['fingerprint'] = self._fingerprint(data)
                return sample
        except (FileNotFoundError, OSError):
            pass
        return None

    def status(self):
        with self.lock:
            running = self.process is not None and self.process.poll() is None
        try:
            age = time.time() - os.path.getmtime(CAMERA_FILE)
        except OSError:
            age = None
        return {
            'running': running,
            'online': running and age is not None and age < 2.5,
            'age': None if age is None else round(age, 2),
        }


class NavigationState:
    """Thread-safe navigation pose, paths, chassis state and A* map."""

    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = False
        self.pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0,
                     'pitch': 0.0, 'yaw': 0.0, 'vx': 0.0, 'vy': 0.0,
                     'vz': 0.0, 'speed': 0.0, 'yaw_rate': 0.0, 't': 0.0}
        self.pose_t = 0.0
        self.lowstate_t = 0.0
        self.lowstate = {
            'imu_rpy': [0.0, 0.0, 0.0], 'gyro': [0.0, 0.0, 0.0],
            'accel': [0.0, 0.0, 0.0], 'soc': None, 'voltage': None,
            'current': None, 'imu_temp': None,
        }
        self.actual = []
        self.global_dense = []
        self.global_waypoints = []
        self.global_progress = 0
        self.local = []
        self.local_t = 0.0
        # SCAN's live collision map is kept in map coordinates for direct
        # browser rendering.  The planner still consumes its full-resolution
        # GridMap; these arrays are visualization-only, bounded samples.
        self.obstacle_raw = np.empty((0, 3), np.float32)
        self.obstacle_raw_t = 0.0
        self.obstacle_raw_source_count = 0
        self.obstacle_inflated = np.empty((0, 3), np.float32)
        self.obstacle_inflated_t = 0.0
        self.obstacle_inflated_source_count = 0
        # Full live inflated cloud retained for one-shot temporary-start
        # selection. The browser still receives only obstacle_inflated's
        # bounded sample, so this does not increase WebSocket bandwidth.
        self.obstacle_inflated_planning = np.empty((0, 3), np.float32)
        self.local_horizon = None
        self.local_horizon_t = 0.0
        self.local_waiting = False
        self.local_waiting_t = 0.0
        self.recovery_status = {
            'active': False, 'action': None, 'reason': '等待导航'}
        self.goal = None
        self.temporary_start = None
        self.route_message = '等待设置目标点'
        self.chassis_enable_callback = None
        self.meta = {}
        self.boot_id = _boot_id()
        # A map/odom transform is valid only while the Web pose consumer and
        # its upstream LIO/IMU chain stay in one continuous run.  Linux boot ID
        # alone is insufficient: restarting the host bridge can make LIO
        # establish a different odom origin without rebooting the computer.
        # A maintenance Web-only reload may explicitly retain the current LIO
        # session while the lidar/odometry graph keeps running.  Normal starts
        # do not set this variable and therefore remain fail-closed with a new
        # random session id.
        self.navigation_session_id = (
            os.environ.get('GO2_NAVIGATION_SESSION_ID') or uuid.uuid4().hex)
        self.map_signature = ''
        self.alignment_valid = False
        self.alignment_reason = '本次开机后尚未标定地图位姿'
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.alignment_data = {}
        self.odom_pose = None
        self.static_points = np.empty((0, 3), np.float32)
        self.free = None
        self.clearance_m = None
        self.clearance_penalty = None
        self.resolution = 0.1
        self.origin = (0.0, 0.0)
        self._load_map()

    def set_chassis_enabled(self, enabled, require_alignment=True):
        if enabled and require_alignment:
            with self.lock:
                if not self.alignment_valid:
                    return False, '请先完成本次开机后的地图位姿标定'
        callback = self.chassis_enable_callback
        if callback is None:
            return False, '底盘安全门尚未就绪'
        callback(bool(enabled))
        return True, ('底盘已请求启用；请确认状态变为“已启用”后再发送目标' if enabled
                      else '底盘已锁定并请求停车')

    def _load_map(self):
        with open(NAV_META, 'r', encoding='utf-8') as stream:
            meta = json.load(stream)
        origin = meta['origin_xy']
        native_res = float(meta['resolution_m'])
        image = _read_pgm(NAV_GRID)
        # PGM row 0 is the north edge; routing coordinates grow from the south edge.
        # The map is already inflated for robot clearance.  The old 2x2
        # all-free reduction added another implicit margin and disconnected
        # narrow but valid passages, so route directly on the native 5 cm grid.
        free = image[::-1] >= 250
        clearance_m = ndimage.distance_transform_edt(
            free, sampling=native_res).astype(np.float32)
        normalized_near_wall = np.clip(
            (GLOBAL_CLEARANCE_SOFT_M - clearance_m) /
            GLOBAL_CLEARANCE_SOFT_M, 0.0, 1.0)
        clearance_penalty = (
            GLOBAL_CLEARANCE_WEIGHT * normalized_near_wall ** 2).astype(
                np.float32)
        # Four-connected components exactly match A* reachability because a
        # diagonal A* step is allowed only when both adjacent cardinal cells
        # are free.  Keep these labels for component-aware target snapping.
        component_structure = np.array(
            [[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)
        free_components, free_component_count = ndimage.label(
            free, structure=component_structure)
        footprint_free = self._build_footprint_masks(free, native_res)
        footprint_any_free = np.any(footprint_free, axis=0)
        # Reject obviously disconnected targets before entering the much more
        # expensive (x, y, heading) search. Eight-connectivity is deliberately
        # an over-approximation of the 16 motion primitives, so it cannot reject
        # a route solely because of the precheck.
        footprint_components, footprint_component_count = ndimage.label(
            footprint_any_free, structure=np.ones((3, 3), dtype=np.uint8))
        footprint_graph_ids, footprint_graph = self._build_footprint_graph(
            footprint_any_free)

        source = meta['source']
        with np.load(source, allow_pickle=False) as data:
            points = np.asarray(data['map'], dtype=np.float32)
        a, b, c = meta['floor_plane_z_ax_by_c']
        points = points.copy()
        # 静态 3D 地图只在导航页首次打开时加载一次，不参与实时推流。
        # 保留原图的全部点，仅将拟合地面调平，避免为实时性做裁剪或抽稀。
        points[:, 2] -= a * points[:, 0] + b * points[:, 1] + c
        signature = _map_signature(meta)
        alignment, alignment_reason = self._read_alignment(meta, signature)
        # Publish one internally consistent map snapshot.
        with self.lock:
            self.meta = meta
            self.free = free
            self.clearance_m = clearance_m
            self.clearance_penalty = clearance_penalty
            self.free_components = free_components
            self.free_component_count = free_component_count
            self.footprint_free = footprint_free
            self.footprint_any_free = footprint_any_free
            self.footprint_components = footprint_components
            self.footprint_component_count = footprint_component_count
            self.footprint_graph_ids = footprint_graph_ids
            # Reverse edges let one goal-rooted Dijkstra call provide an exact
            # obstacle-aware lower bound for every possible start position.
            self.footprint_graph_reverse = footprint_graph.transpose().tocsr()
            self.footprint_heuristic_goal = -1
            self.footprint_heuristic_costs = None
            self.resolution = native_res
            self.origin = (float(origin[0]), float(origin[1]))
            self.static_points = points
            self.map_signature = signature
            if alignment is None:
                self.alignment_valid = False
                self.alignment_reason = alignment_reason
                self.map_to_odom = (0.0, 0.0, 0.0)
                self.alignment_data = {}
            else:
                transform = alignment['map_to_odom']
                self.map_to_odom = (
                    float(transform[0]), float(transform[1]), float(transform[3]))
                self.alignment_valid = True
                self.alignment_reason = '已完成本次开机位姿标定'
                self.alignment_data = alignment


    def _read_alignment(self, meta, signature):
        try:
            with open(NAV_ALIGNMENT, 'r', encoding='utf-8') as stream:
                alignment = json.load(stream)
        except FileNotFoundError:
            return None, '本次开机后尚未标定地图位姿'
        except (OSError, ValueError, TypeError):
            return None, '位姿标定文件无效'
        if alignment.get('boot_id') != self.boot_id:
            return None, '机器已重启，需重新标定地图位姿'
        if alignment.get('navigation_session_id') != self.navigation_session_id:
            # A second Web process can be started briefly during a host-node
            # restart and fail later because port 8890 is already owned by the
            # live service.  Such a process must not erase the live service's
            # verified alignment merely because it has a different session.
            # Keep the file, reject it only for this process, and remain
            # fail-closed until a matching session or a new calibration exists.
            return None, 'Web/LIO会话已重启，需重新自动定位'
        if alignment.get('map_signature') != signature:
            return None, '导航基准图已变更，需重新标定位姿'
        transform = alignment.get('map_to_odom')
        if (not isinstance(transform, list) or len(transform) != 4 or
                not all(math.isfinite(float(value)) for value in transform)):
            return None, '位姿标定变换无效'
        if os.path.realpath(str(alignment.get('map_source', ''))) != os.path.realpath(
                str(meta.get('source', ''))):
            return None, '导航地图不匹配，需重新标定位姿'
        return alignment, ''

    @staticmethod
    def _wrap_angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    @staticmethod
    def _map_to_odom_xy(x, y, transform):
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        return c * x - s * y + tx, s * x + c * y + ty

    @staticmethod
    def _odom_to_map_xy(x, y, transform):
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - tx, y - ty
        return c * dx + s * dy, -s * dx + c * dy

    def is_alignment_valid(self):
        with self.lock:
            return bool(self.alignment_valid)

    def memory_map_snapshot(self):
        """Return the immutable identity used to isolate semantic memories."""
        with self.lock:
            if not self.map_signature:
                return None, '导航基准图尚未加载'
            return {
                'map_signature': self.map_signature,
                'map_source': os.path.basename(self.meta.get('source', '')),
                'navigation_session_id': self.navigation_session_id,
            }, ''

    def memory_pose_snapshot(self, max_age=1.0):
        """Return a fresh, map-aligned pose or fail closed for memory capture."""
        with self.lock:
            if not self.enabled:
                return None, '当前不在导航模式'
            if not self.alignment_valid:
                return None, self.alignment_reason or '地图位姿尚未标定'
            now = time.time()
            age = now - self.pose_t if self.pose_t else None
            if age is None or age > float(max_age):
                return None, '地图位姿数据过期'
            pose = {
                'x': float(self.pose['x']),
                'y': float(self.pose['y']),
                'z': float(self.pose['z']),
                'yaw': float(self.pose['yaw']),
                'observed_at': float(self.pose_t),
                'map_signature': self.map_signature,
                'map_source': os.path.basename(self.meta.get('source', '')),
                'navigation_session_id': self.navigation_session_id,
            }
            return pose, ''

    def alignment_status(self):
        with self.lock:
            result = {
                'valid': bool(self.alignment_valid),
                'reason': self.alignment_reason,
                'map_source': os.path.basename(self.meta.get('source', '')),
            }
            if self.alignment_valid:
                result['map_pose'] = dict(self.alignment_data.get('map_pose', {}))
                result['map_to_odom'] = [round(value, 6) for value in (
                    self.map_to_odom[0], self.map_to_odom[1], 0.0,
                    self.map_to_odom[2])]
                result['method'] = self.alignment_data.get('method', 'manual')
                result['quality'] = dict(self.alignment_data.get('quality', {}))
            return result

    def nav2_map_to_odom_tf(self):
        """Return the standard ROS ``nav_map -> odom`` TF transform.

        ``map_to_odom`` is the historical planner transform and evaluates
        ``p_odom = T_odom_map * p_map``.  A ROS TF whose parent is ``nav_map``
        and child is ``odom`` stores the inverse transform, so do that inversion
        in one tested owner rather than duplicating it in launch-side bridges.
        """
        with self.lock:
            if not self.alignment_valid:
                return None
            tx, ty, yaw = self.map_to_odom
        c, s = math.cos(yaw), math.sin(yaw)
        return {
            'x': -c * tx - s * ty,
            'y': s * tx - c * ty,
            'z': 0.0,
            'yaw': self._wrap_angle(-yaw),
        }

    def set_alignment(self, map_x, map_y, map_yaw, method='manual', quality=None,
                      rough_pose=None):
        values = (float(map_x), float(map_y), float(map_yaw))
        if not all(math.isfinite(value) for value in values):
            return False, '标定位姿无效'
        with self.lock:
            if self.odom_pose is None or time.time() - self.pose_t > 2.5:
                return False, '导航里程计离线，无法标定'
            odom = dict(self.odom_pose)
            signature = self.map_signature
            source = os.path.realpath(str(self.meta.get('source', '')))
        transform_yaw = self._wrap_angle(odom['yaw'] - values[2])
        c, s = math.cos(transform_yaw), math.sin(transform_yaw)
        tx = odom['x'] - (c * values[0] - s * values[1])
        ty = odom['y'] - (s * values[0] + c * values[1])
        alignment = {
            'schema': 1,
            'boot_id': self.boot_id,
            'navigation_session_id': self.navigation_session_id,
            'map_signature': signature,
            'map_source': source,
            'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'map_pose': {
                'x': values[0], 'y': values[1], 'yaw': values[2],
                'yaw_deg': math.degrees(values[2]),
            },
            'odom_pose': {
                'x': odom['x'], 'y': odom['y'], 'yaw': odom['yaw'],
            },
            'map_to_odom': [tx, ty, 0.0, transform_yaw],
            'method': str(method),
            'quality': dict(quality or {}),
            'rough_pose': dict(rough_pose or {}),
        }
        temporary = NAV_ALIGNMENT + '.tmp-%d' % os.getpid()
        try:
            with open(temporary, 'w', encoding='utf-8') as stream:
                json.dump(alignment, stream, indent=2, ensure_ascii=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, NAV_ALIGNMENT)
        except OSError as exc:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            return False, '位姿标定保存失败: %s' % exc
        with self.lock:
            self.map_to_odom = (tx, ty, transform_yaw)
            self.alignment_valid = True
            self.alignment_reason = '已完成本次开机位姿标定'
            self.alignment_data = alignment
            self.actual = []
            self.global_dense = []
            self.global_waypoints = []
            self.global_progress = 0
            self.local = []
            self.local_waiting = False
            self.local_waiting_t = 0.0
            self.goal = None
            self.temporary_start = None
            self.route_message = '位姿已标定，请确认地图中机器人位置和朝向'
            self._refresh_map_pose_locked()
        return True, '位姿标定已保存'

    def invalidate_alignment(self, reason='正在重新自动定位'):
        """Fail closed before any new localization attempt."""
        try:
            os.unlink(NAV_ALIGNMENT)
        except FileNotFoundError:
            pass
        except OSError:
            # In-memory invalidation is the safety boundary; a stale on-disk
            # file is still boot/map checked and will be overwritten on success.
            pass
        with self.lock:
            self._invalidate_alignment_locked(reason)
            self._refresh_map_pose_locked()

    def _invalidate_alignment_locked(self, reason):
        """Invalidate map alignment while ``self.lock`` is already held."""
        self.alignment_valid = False
        self.alignment_reason = str(reason)
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.alignment_data = {}
        self.actual = []
        self.global_dense = []
        self.global_waypoints = []
        self.global_progress = 0
        self.local = []
        self.local_t = 0.0
        self.local_waiting = False
        self.local_waiting_t = 0.0
        self.goal = None
        self.temporary_start = None
        self.route_message = str(reason)

    def _refresh_map_pose_locked(self):
        if self.odom_pose is None:
            return
        odom = self.odom_pose
        if self.alignment_valid:
            x, y = self._odom_to_map_xy(odom['x'], odom['y'], self.map_to_odom)
            yaw = self._wrap_angle(odom['yaw'] - self.map_to_odom[2])
            c, s = math.cos(self.map_to_odom[2]), math.sin(self.map_to_odom[2])
            vx = c * odom['vx'] + s * odom['vy']
            vy = -s * odom['vx'] + c * odom['vy']
        else:
            x, y, yaw = odom['x'], odom['y'], odom['yaw']
            vx, vy = odom['vx'], odom['vy']
        self.pose.update({
            'x': x, 'y': y, 'z': odom['z'], 'roll': odom['roll'],
            'pitch': odom['pitch'], 'yaw': yaw, 'vx': vx, 'vy': vy,
            'vz': odom['vz'], 'speed': odom['speed'], 't': odom['t'],
        })

    def reload_map(self):
        """Reload atomically promoted navigation artifacts while navigation is idle."""
        self._load_map()

    def static_map_payload(self):
        return {
            'points': _flat(self.static_points),
            'point_count': int(len(self.static_points)),
            'frame_id': 'map',
            'bounds': {
                'origin': list(self.origin),
                'width': int(self.free.shape[1]),
                'height': int(self.free.shape[0]),
                'resolution': self.resolution,
            },
            'source': os.path.basename(self.meta.get('source', '')),
        }

    def set_enabled(self, enabled):
        with self.lock:
            self.enabled = bool(enabled)
            if enabled:
                self.actual = []
                self.global_dense = []
                self.global_waypoints = []
                self.global_progress = 0
                self.local = []
                self.local_t = 0.0
                self.local_waiting = False
                self.local_waiting_t = 0.0
                self.goal = None
                self.temporary_start = None
                self.route_message = '等待设置目标点'

    def update_pose(self, msg, rpy):
        now = time.time()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        roll, pitch, yaw = rpy
        alignment_jump_reason = None
        alignment_callback = None
        with self.lock:
            previous_odom = self.odom_pose
            old_yaw = previous_odom['yaw'] if previous_odom is not None else yaw
            dt = now - self.pose_t if self.pose_t else 0.0
            yaw_delta = math.atan2(math.sin(yaw - old_yaw), math.cos(yaw - old_yaw))
            yaw_rate = yaw_delta / dt if 0.02 < dt < 1.0 else self.pose['yaw_rate']
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            if (self.alignment_valid and previous_odom is not None and
                    0.02 < dt < 1.0):
                position_jump = math.hypot(
                    float(p.x) - previous_odom['x'],
                    float(p.y) - previous_odom['y'])
                # LIO-SAM normally publishes only about 4--5 Hz and can leave
                # a much longer gap while its graph is busy. At the physical
                # 0.8 m/s ceiling, a valid update after such a gap can exceed
                # the old fixed 0.50 m threshold. Scale the discontinuity gate
                # with elapsed time while retaining a hard upper bound for a
                # genuine odom-frame reset.
                position_jump_limit = min(
                    1.25, max(0.50, 0.20 + 1.20 * dt))
                # At the physical gate limits a valid sample-to-sample motion
                # is only a few centimetres/degrees.  A much larger discontinuity
                # means LIO changed its odom frame (for example after a graph
                # correction or an IMU outage), so the persisted map transform
                # is no longer meaningful even though Linux did not reboot.
                if (position_jump > position_jump_limit or
                        abs(yaw_delta) > math.radians(45.0)):
                    alignment_jump_reason = (
                        '检测到LIO坐标系跳变 %.2f m / %.1f° '
                        '(帧间隔 %.2fs，位移上限 %.2fm)，需重新自动定位' % (
                            position_jump, math.degrees(abs(yaw_delta)),
                            dt, position_jump_limit))
                    self._invalidate_alignment_locked(alignment_jump_reason)
                    alignment_callback = self.chassis_enable_callback
            self.odom_pose = {
                'x': float(p.x), 'y': float(p.y), 'z': float(p.z),
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'vx': float(v.x), 'vy': float(v.y), 'vz': float(v.z),
                'speed': speed, 'yaw_rate': yaw_rate, 't': now,
            }
            self.pose['yaw_rate'] = yaw_rate
            self._refresh_map_pose_locked()
            self.pose_t = now
            if self.enabled and self.alignment_valid:
                point = (self.pose['x'], self.pose['y'], max(0.04, self.pose['z'] - 0.40))
                if not self.actual or math.hypot(
                        point[0] - self.actual[-1][0], point[1] - self.actual[-1][1]) >= 0.03:
                    self.actual.append(point)
                    if len(self.actual) > 10000:
                        self.actual = self.actual[-10000:]
                self._advance_progress_locked(point[0], point[1])
        if alignment_jump_reason is not None:
            # The in-memory flag above is the immediate safety boundary.  Also
            # remove the persisted transform and explicitly lock the actuator
            # gate so neither can be reused after a Web/SCAN restart.
            try:
                os.unlink(NAV_ALIGNMENT)
            except (FileNotFoundError, OSError):
                pass
            if alignment_callback is not None:
                alignment_callback(False)

    def _advance_progress_locked(self, x, y):
        if not self.global_dense:
            return
        start = max(0, self.global_progress - 3)
        end = min(len(self.global_dense), self.global_progress + 120)
        nearest = min(range(start, end), key=lambda i: (
            self.global_dense[i][0] - x) ** 2 + (self.global_dense[i][1] - y) ** 2)
        self.global_progress = max(self.global_progress, nearest)

    def update_lowstate(self, msg):
        imu = getattr(msg, 'imu_state', None)
        bms = getattr(msg, 'bms_state', None)

        def triple(obj, name):
            value = list(getattr(obj, name, [])) if obj is not None else []
            return [float(value[i]) if i < len(value) else 0.0 for i in range(3)]

        with self.lock:
            self.lowstate = {
                'imu_rpy': triple(imu, 'rpy'),
                'gyro': triple(imu, 'gyroscope'),
                'accel': triple(imu, 'accelerometer'),
                'soc': int(getattr(bms, 'soc', 0)) if bms is not None else None,
                'voltage': float(getattr(msg, 'power_v', 0.0)),
                'current': float(getattr(msg, 'power_a', 0.0)),
                'imu_temp': int(getattr(imu, 'temperature', 0)) if imu is not None else None,
            }
            self.lowstate_t = time.time()

    def update_local_path(self, msg):
        with self.lock:
            if not self.alignment_valid:
                self.local = []
                self.local_t = 0.0
                return
            transform = self.map_to_odom
            points = []
            for pose in msg.poses:
                p = pose.pose.position
                x, y = self._odom_to_map_xy(p.x, p.y, transform)
                points.append((x, y, p.z))
            self.local = points
            self.local_t = time.time()

    def update_obstacle_cloud(self, points, inflated, source_count,
                              planning_points=None):
        """Store a bounded SCAN occupancy sample in the static-map frame."""
        cloud = np.asarray(points, dtype=np.float32)
        if cloud.ndim != 2 or cloud.shape[1] < 3:
            return
        cloud = cloud[:, :3].copy()
        planning_cloud = None
        if inflated:
            source = points if planning_points is None else planning_points
            planning_cloud = np.asarray(source, dtype=np.float32)
            if planning_cloud.ndim != 2 or planning_cloud.shape[1] < 3:
                return
            planning_cloud = planning_cloud[:, :3].copy()
        now = time.time()
        with self.lock:
            if not self.alignment_valid:
                if inflated:
                    self.obstacle_inflated = np.empty((0, 3), np.float32)
                    self.obstacle_inflated_t = 0.0
                    self.obstacle_inflated_source_count = 0
                    self.obstacle_inflated_planning = np.empty((0, 3), np.float32)
                else:
                    self.obstacle_raw = np.empty((0, 3), np.float32)
                    self.obstacle_raw_t = 0.0
                    self.obstacle_raw_source_count = 0
                return
            tx, ty, yaw = self.map_to_odom
            c, s = math.cos(yaw), math.sin(yaw)
            dx = cloud[:, 0] - tx
            dy = cloud[:, 1] - ty
            cloud[:, 0] = c * dx + s * dy
            cloud[:, 1] = -s * dx + c * dy
            if inflated:
                planning_dx = planning_cloud[:, 0] - tx
                planning_dy = planning_cloud[:, 1] - ty
                planning_cloud[:, 0] = c * planning_dx + s * planning_dy
                planning_cloud[:, 1] = -s * planning_dx + c * planning_dy
                self.obstacle_inflated = cloud
                self.obstacle_inflated_planning = planning_cloud
                self.obstacle_inflated_t = now
                self.obstacle_inflated_source_count = int(source_count)
            else:
                self.obstacle_raw = cloud
                self.obstacle_raw_t = now
                self.obstacle_raw_source_count = int(source_count)

    def update_local_horizon(self, msg):
        """Track the exact start/target pair used by the current SCAN attempt."""
        if len(msg.poses) < 2:
            with self.lock:
                self.local_horizon = None
                self.local_horizon_t = 0.0
            return
        with self.lock:
            if not self.alignment_valid:
                self.local_horizon = None
                self.local_horizon_t = 0.0
                return
            transform = self.map_to_odom
            horizon = []
            for stamped in msg.poses[:2]:
                point = stamped.pose.position
                x, y = self._odom_to_map_xy(point.x, point.y, transform)
                horizon.append((x, y, float(point.z)))
            self.local_horizon = horizon
            self.local_horizon_t = time.time()

    def update_local_waiting(self, msg):
        """Track SCAN's persistent wait/retry state for the browser."""
        with self.lock:
            self.local_waiting = bool(msg.data)
            self.local_waiting_t = time.time()
            if self.local_waiting:
                self.local = []
                self.local_t = 0.0

    def update_recovery_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.recovery_status = status

    def is_local_waiting(self):
        with self.lock:
            return bool(self.local_waiting and self.goal is not None)

    def clear_navigation(self, message='导航已停止'):
        with self.lock:
            self.global_dense = []
            self.global_waypoints = []
            self.global_progress = 0
            self.local = []
            self.local_t = 0.0
            self.local_horizon = None
            self.local_horizon_t = 0.0
            self.local_waiting = False
            self.local_waiting_t = 0.0
            self.goal = None
            self.temporary_start = None
            self.route_message = message

    def has_active_goal(self):
        with self.lock:
            return self.goal is not None

    def snapshot(self, camera_status, planner_active, chassis_status=None,
                 with_paths=True, with_obstacles=True):
        with self.lock:
            now = time.time()
            pose_age = now - self.pose_t if self.pose_t else None
            low_age = now - self.lowstate_t if self.lowstate_t else None
            local_age = now - self.local_t if self.local_t else None
            raw_age = now - self.obstacle_raw_t if self.obstacle_raw_t else None
            inflated_age = (now - self.obstacle_inflated_t
                            if self.obstacle_inflated_t else None)
            horizon_age = now - self.local_horizon_t if self.local_horizon_t else None
            raw_online = raw_age is not None and raw_age < 1.5
            inflated_online = inflated_age is not None and inflated_age < 1.5
            horizon_online = (horizon_age is not None and horizon_age < 1.5 and
                              self.local_horizon is not None and self.goal is not None)
            remaining = self.global_dense[self.global_progress:]
            route_message = self.route_message
            recovery = dict(self.recovery_status)
            if recovery.get('active') and self.goal is not None:
                route_message += ' · 实时点云脱困：%s' % (
                    recovery.get('action') or '执行中')
            elif self.local_waiting and self.goal is not None:
                route_message += ' · 实时障碍阻塞，原地等待并自动重试'
            return {
                'mode': 'navigation',
                'pose': dict(self.pose),
                'actual_path': _flat(self.actual) if with_paths else None,
                'global_path': _flat(remaining) if with_paths else None,
                'local_path': (_flat(self.local) if local_age is not None and local_age < 2.0 else [])
                if with_paths else None,
                'local_obstacles': {
                    'raw': (_flat(self.obstacle_raw) if with_obstacles and raw_online else
                            ([] if with_obstacles else None)),
                    'inflated': (_flat(self.obstacle_inflated)
                                 if with_obstacles and inflated_online else
                                 ([] if with_obstacles else None)),
                    'raw_sample_count': int(len(self.obstacle_raw)) if raw_online else 0,
                    'raw_source_count': (self.obstacle_raw_source_count
                                         if raw_online else 0),
                    'raw_age': None if raw_age is None else round(raw_age, 2),
                    'raw_online': raw_online,
                    'inflated_sample_count': (int(len(self.obstacle_inflated))
                                              if inflated_online else 0),
                    'inflated_source_count': (self.obstacle_inflated_source_count
                                              if inflated_online else 0),
                    'inflated_age': (None if inflated_age is None else
                                     round(inflated_age, 2)),
                    'inflated_online': inflated_online,
                    'start': (_flat([self.local_horizon[0]])
                              if horizon_online else None),
                    'target': (_flat([self.local_horizon[1]])
                               if horizon_online else None),
                    'horizon_age': (None if horizon_age is None else
                                    round(horizon_age, 2)),
                    'horizon_online': horizon_online,
                    'sampled_for_web': True,
                },
                'goal': self.goal,
                'temporary_start': (None if self.temporary_start is None else
                                    dict(self.temporary_start)),
                'route_message': route_message,
                'state': dict(self.lowstate),
                'health': {
                    'pose_online': pose_age is not None and pose_age < 1.0,
                    'pose_age': None if pose_age is None else round(pose_age, 2),
                    'lowstate_online': low_age is not None and low_age < 1.5,
                    'lowstate_age': None if low_age is None else round(low_age, 2),
                    'local_path_online': local_age is not None and local_age < 2.0,
                    'local_waiting': bool(self.local_waiting and self.goal is not None),
                    'recovery': recovery,
                    'planner_active': planner_active,
                    'camera': camera_status,
                    'dry_run': not bool((chassis_status or {}).get('enabled')),
                    'chassis': dict(chassis_status or {}),
                    'alignment': self.alignment_status_locked(),
                },
            }

    def alignment_status_locked(self):
        result = {
            'valid': bool(self.alignment_valid),
            'reason': self.alignment_reason,
            'map_source': os.path.basename(self.meta.get('source', '')),
        }
        if self.alignment_valid:
            result['map_pose'] = dict(self.alignment_data.get('map_pose', {}))
            result['method'] = self.alignment_data.get('method', 'manual')
            result['quality'] = dict(self.alignment_data.get('quality', {}))
        return result

    def validate_alignment_pose(self, x, y):
        """Accept a map-error start only when a nearby static exit exists."""
        cell = self._to_cell(float(x), float(y))
        height, width = self.free.shape
        if not (0 <= cell[0] < width and 0 <= cell[1] < height):
            return False, '自动定位结果超出导航地图'
        if not self.free[cell[1], cell[0]]:
            if self._temporary_start_candidates((float(x), float(y))) is None:
                return False, '自动定位结果位于静态障碍内，且附近没有可用临时起点'
            return True, '自动定位结果位于静态障碍内，规划时将使用临时起点'
        return True, ''

    def _to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def _to_world(self, cell):
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)

    def _nearest_free(self, cell, max_distance=1.8, component=None):
        x, y = cell
        height, width = self.free.shape
        max_radius = max(0, int(math.ceil(max_distance / self.resolution)))
        x0, x1 = max(0, x - max_radius), min(width - 1, x + max_radius)
        y0, y1 = max(0, y - max_radius), min(height - 1, y + max_radius)
        if x0 > x1 or y0 > y1:
            return None
        valid = self.free[y0:y1 + 1, x0:x1 + 1].copy()
        if component is not None:
            valid &= (self.free_components[y0:y1 + 1, x0:x1 + 1] == component)
        ys, xs = np.nonzero(valid)
        if not len(xs):
            return None
        xs = xs + x0
        ys = ys + y0
        distance2 = np.square(xs - x) + np.square(ys - y)
        inside = distance2 <= max_radius * max_radius
        if not inside.any():
            return None
        choices = np.flatnonzero(inside)
        best = choices[int(np.argmin(distance2[inside]))]
        return int(xs[best]), int(ys[best])

    def _temporary_start_candidates(self, start_xy):
        """Return saved-grid free cells in the configured escape annulus."""
        centre = self._to_cell(*start_xy)
        height, width = self.free.shape
        radius = int(math.ceil(TEMP_START_MAX_RADIUS_M / self.resolution))
        x0, x1 = max(0, centre[0] - radius), min(width - 1, centre[0] + radius)
        y0, y1 = max(0, centre[1] - radius), min(height - 1, centre[1] + radius)
        if x0 > x1 or y0 > y1:
            return None
        ys, xs = np.nonzero(self.free[y0:y1 + 1, x0:x1 + 1])
        if not len(xs):
            return None
        xs = xs + x0
        ys = ys + y0
        wx = self.origin[0] + (xs.astype(np.float64) + 0.5) * self.resolution
        wy = self.origin[1] + (ys.astype(np.float64) + 0.5) * self.resolution
        distances = np.hypot(wx - start_xy[0], wy - start_xy[1])
        keep = ((distances >= TEMP_START_MIN_RADIUS_M) &
                (distances <= TEMP_START_MAX_RADIUS_M))
        if not keep.any():
            return None
        return (xs[keep], ys[keep], wx[keep], wy[keep], distances[keep])

    def _select_temporary_start(self, start_xy, raw_goal, live_inflated):
        """Select a live-clear static entry point that can reach the goal."""
        candidates = self._temporary_start_candidates(start_xy)
        if candidates is None:
            return None
        xs, ys, wx, wy, distances = candidates
        components = self.free_components[ys, xs]

        # Only retain static components onto which the requested target can be
        # snapped. This avoids escaping into a small free island that cannot
        # participate in the requested global route.
        goals = {}
        for component in np.unique(components):
            component = int(component)
            if component > 0:
                goal = self._nearest_free(raw_goal, component=component)
                if goal is not None:
                    goals[component] = goal
        if not goals:
            return None
        viable = np.fromiter(
            (int(component) in goals for component in components),
            dtype=np.bool_, count=len(components))

        live_xy = np.asarray(live_inflated, dtype=np.float32)
        if live_xy.ndim != 2 or live_xy.shape[1] < 2:
            return None
        if len(live_xy):
            live_distances, _ = cKDTree(live_xy[:, :2]).query(
                np.column_stack((wx, wy)), k=1)
        else:
            live_distances = np.full(len(wx), np.inf, dtype=np.float64)
        # occupancy_inflate is a voxel-centre cloud that is already expanded
        # by SCAN's collision radius. Half a cell diagonal distinguishes a
        # genuinely different free cell without adding another large radius.
        live_hard_clearance = self.resolution * 0.75
        viable &= live_distances > live_hard_clearance
        choices = np.flatnonzero(viable)
        if not len(choices):
            return None

        static_clearance = self.clearance_m[ys, xs].astype(np.float64)
        combined_clearance = np.minimum(static_clearance, live_distances)
        open_choices = choices[
            combined_clearance[choices] >= TEMP_START_OPEN_CLEARANCE_M]
        if len(open_choices):
            # Once the requested openness is met, prefer the closest point so
            # a map error does not create an unnecessarily long escape leg.
            score = (distances[open_choices] -
                     0.10 * np.minimum(combined_clearance[open_choices], 0.8))
            best = int(open_choices[int(np.argmin(score))])
        else:
            # A narrow corridor may not contain 20 cm of extra clearance.
            # Retain generality by choosing the best available clearance while
            # still penalizing a needlessly distant point.
            score = (2.0 * np.minimum(combined_clearance[choices], 0.8) -
                     0.75 * distances[choices])
            best = int(choices[int(np.argmax(score))])

        component = int(components[best])
        return {
            'cell': (int(xs[best]), int(ys[best])),
            'goal': goals[component],
            'world': (float(wx[best]), float(wy[best])),
            'distance': float(distances[best]),
            'static_clearance': float(static_clearance[best]),
            'live_clearance': float(live_distances[best]),
        }

    def _goal_on_start_component(self, raw_goal, start):
        component = int(self.free_components[start[1], start[0]])
        if not component:
            return None
        return self._nearest_free(raw_goal, component=component)

    @staticmethod
    def _build_footprint_masks(free, resolution):
        """Return one double-cylinder validity mask per motion heading."""
        height, width = free.shape
        masks = []
        offset_cells = FOOTPRINT_OFFSET_M / resolution
        for dx, dy in FOOTPRINT_DIRECTIONS:
            norm = math.hypot(dx, dy)
            ox = int(round(offset_cells * dx / norm))
            oy = int(round(offset_cells * dy / norm))
            x0, x1 = max(0, -ox, ox), min(width, width - ox, width + ox)
            y0, y1 = max(0, -oy, oy), min(height, height - oy, height + oy)
            mask = np.zeros_like(free, dtype=np.bool_)
            if x0 < x1 and y0 < y1:
                mask[y0:y1, x0:x1] = (
                    free[y0:y1, x0:x1] &
                    free[y0 + oy:y1 + oy, x0 + ox:x1 + ox] &
                    free[y0 - oy:y1 - oy, x0 - ox:x1 - ox])
            masks.append(mask)
        return np.stack(masks, axis=0)

    @staticmethod
    def _build_footprint_graph(footprint_any_free):
        """Build a sparse position relaxation containing every real move.

        The graph ignores heading, so its goal distances can only
        underestimate the full state-lattice cost.  Unlike straight-line
        distance, it already accounts for walls and large-map detours.
        """
        height, width = footprint_any_free.shape
        node_ids = np.full(height * width, -1, dtype=np.int32)
        free_linear = np.flatnonzero(footprint_any_free.ravel())
        node_ids[free_linear] = np.arange(free_linear.size, dtype=np.int32)
        node_grid = node_ids.reshape(height, width)
        rows, columns, weights = [], [], []

        for dx, dy in FOOTPRINT_DIRECTIONS:
            x0, x1 = max(0, -dx), min(width, width - dx)
            y0, y1 = max(0, -dy), min(height, height - dy)
            if x0 >= x1 or y0 >= y1:
                continue
            valid = footprint_any_free[y0:y1, x0:x1].copy()
            for crossed_x, crossed_y in NavigationState._primitive_cells(
                    0, 0, dx, dy):
                valid &= footprint_any_free[
                    y0 + crossed_y:y1 + crossed_y,
                    x0 + crossed_x:x1 + crossed_x]
            source = node_grid[y0:y1, x0:x1][valid]
            target = node_grid[y0 + dy:y1 + dy, x0 + dx:x1 + dx][valid]
            if source.size:
                rows.append(source)
                columns.append(target)
                weights.append(np.full(source.size, math.hypot(dx, dy),
                                       dtype=np.float32))

        if not rows:
            graph = sparse.csr_matrix((free_linear.size, free_linear.size),
                                      dtype=np.float32)
        else:
            graph = sparse.csr_matrix(
                (np.concatenate(weights),
                 (np.concatenate(rows), np.concatenate(columns))),
                shape=(free_linear.size, free_linear.size), dtype=np.float32)
            graph.sum_duplicates()
        return node_ids.reshape(height, width), graph

    def _goal_cost_heuristic(self, goal):
        """Obstacle-aware lower-bound cost from every map position to goal."""
        goal_linear = goal[1] * self.free.shape[1] + goal[0]
        goal_node = int(self.footprint_graph_ids.ravel()[goal_linear])
        if goal_node < 0:
            return None
        if (self.footprint_heuristic_goal != goal_node or
                self.footprint_heuristic_costs is None):
            self.footprint_heuristic_costs = csgraph.dijkstra(
                self.footprint_graph_reverse, directed=True,
                indices=goal_node, return_predecessors=False)
            self.footprint_heuristic_goal = goal_node
        return self.footprint_heuristic_costs

    @staticmethod
    def _heading_index(yaw):
        return min(range(len(FOOTPRINT_DIRECTIONS)), key=lambda index: abs(
            math.atan2(
                math.sin(math.atan2(FOOTPRINT_DIRECTIONS[index][1],
                                    FOOTPRINT_DIRECTIONS[index][0]) - yaw),
                math.cos(math.atan2(FOOTPRINT_DIRECTIONS[index][1],
                                    FOOTPRINT_DIRECTIONS[index][0]) - yaw))))

    def _nearest_footprint_free(self, cell, max_distance=1.8, component=None,
                                heading=None, component_labels=None):
        x, y = cell
        height, width = self.free.shape
        max_radius = max(0, int(math.ceil(max_distance / self.resolution)))
        x0, x1 = max(0, x - max_radius), min(width - 1, x + max_radius)
        y0, y1 = max(0, y - max_radius), min(height - 1, y + max_radius)
        if x0 > x1 or y0 > y1:
            return None
        source = (self.footprint_any_free if heading is None else
                  self.footprint_free[heading])
        valid = source[y0:y1 + 1, x0:x1 + 1].copy()
        if component is not None:
            labels = (self.free_components if component_labels is None else
                      component_labels)
            valid &= (labels[y0:y1 + 1, x0:x1 + 1] == component)
        ys, xs = np.nonzero(valid)
        if not len(xs):
            return None
        xs += x0
        ys += y0
        distance2 = np.square(xs - x) + np.square(ys - y)
        inside = distance2 <= max_radius * max_radius
        if not inside.any():
            return None
        choices = np.flatnonzero(inside)
        best = choices[int(np.argmin(distance2[inside]))]
        return int(xs[best]), int(ys[best])

    @staticmethod
    def _primitive_cells(x, y, dx, dy):
        """Cells crossed by a short 16-heading motion primitive."""
        # The shallow/steep primitives cross a cell boundary exactly halfway.
        # Treat both touching cells as occupied candidates (supercover), rather
        # than relying on Python's parity-dependent rounding of ``n + 0.5``.
        if abs(dx) == 2 and abs(dy) == 1:
            middle_x = x + dx // 2
            return [(middle_x, y), (middle_x, y + dy), (x + dx, y + dy)]
        if abs(dx) == 1 and abs(dy) == 2:
            middle_y = y + dy // 2
            return [(x, middle_y), (x + dx, middle_y), (x + dx, y + dy)]
        steps = max(abs(dx), abs(dy))
        cells = []
        for step in range(1, steps + 1):
            cell = (int(round(x + dx * step / steps)),
                    int(round(y + dy * step / steps)))
            if not cells or cell != cells[-1]:
                cells.append(cell)
        return cells

    def _footprint_a_star(self, start, goal, start_yaw=None,
                          timeout=FOOTPRINT_ASTAR_TIMEOUT):
        """Full-map state-lattice A* with an obstacle-aware global heuristic."""
        deadline = time.monotonic() + max(0.1, float(timeout))
        height, width = self.free.shape
        heading_count = len(FOOTPRINT_DIRECTIONS)
        cell_count = height * width
        state_count = heading_count * cell_count
        costs = np.full(state_count, np.inf, dtype=np.float32)
        closed = np.zeros(state_count, dtype=np.bool_)
        parents = np.full(state_count, -1, dtype=np.int32)
        graph_ids = self.footprint_graph_ids.ravel()
        obstacle_cost = self._goal_cost_heuristic(goal)
        queue = []

        def heuristic(position, x, y):
            if obstacle_cost is not None:
                graph_node = int(graph_ids[position])
                if graph_node >= 0:
                    value = float(obstacle_cost[graph_node])
                    if math.isfinite(value):
                        return value
            return math.hypot(goal[0] - x, goal[1] - y)

        if start_yaw is None:
            start_headings = range(heading_count)
        else:
            start_headings = (self._heading_index(start_yaw),)
        start_position = start[1] * width + start[0]
        for heading in start_headings:
            if not self.footprint_free[heading, start[1], start[0]]:
                continue
            state = heading * cell_count + start_position
            costs[state] = 0.0
            heapq.heappush(queue, (
                FOOTPRINT_HEURISTIC_WEIGHT *
                heuristic(start_position, start[0], start[1]), -0.0, state))
        if not queue:
            return None

        end_state = -1
        expanded = 0
        while queue:
            _, negative_cost, state = heapq.heappop(queue)
            cost = -negative_cost
            if closed[state] or cost > costs[state] + 1e-5:
                continue
            heading, position = divmod(state, cell_count)
            y, x = divmod(position, width)
            expanded += 1
            if expanded % FOOTPRINT_ASTAR_YIELD_INTERVAL == 0:
                if time.monotonic() >= deadline:
                    raise FootprintSearchTimeout(
                        'footprint A* exceeded %.1fs after %d states' %
                        (timeout, expanded))
                # The planner runs in Tornado's worker pool. Explicitly yield
                # the GIL so status/WebSocket traffic stays responsive during
                # a large but still bounded search.
                time.sleep(0)
            if (x, y) == goal:
                end_state = state
                break
            closed[state] = True

            # Rotate through adjacent headings at the same centre.  Requiring
            # every intermediate mask prevents a corner from sweeping through
            # a wall even if both straight segments are individually free.
            for next_heading in ((heading - 1) % heading_count,
                                 (heading + 1) % heading_count):
                if not self.footprint_free[next_heading, y, x]:
                    continue
                next_state = next_heading * cell_count + position
                candidate = cost + 0.25
                if candidate + 1e-5 < costs[next_state]:
                    costs[next_state] = candidate
                    parents[next_state] = state
                    heapq.heappush(queue, (
                        candidate + FOOTPRINT_HEURISTIC_WEIGHT *
                        heuristic(position, x, y),
                        -candidate, next_state))

            dx, dy = FOOTPRINT_DIRECTIONS[heading]
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            crossed = self._primitive_cells(x, y, dx, dy)
            if not all(self.footprint_free[heading, cy, cx]
                       for cx, cy in crossed):
                continue
            next_position = ny * width + nx
            next_state = heading * cell_count + next_position
            step_distance = math.hypot(dx, dy)
            # Use the closest part of this motion primitive for the soft
            # clearance cost.  This avoids corner cutting beside a wall, but
            # never turns a valid cell into a hard obstacle.
            near_wall_penalty = max(
                float(self.clearance_penalty[cy, cx]) for cx, cy in crossed)
            candidate = cost + step_distance * (1.0 + near_wall_penalty)
            if candidate + 1e-5 < costs[next_state]:
                costs[next_state] = candidate
                parents[next_state] = state
                heapq.heappush(queue, (
                    candidate + FOOTPRINT_HEURISTIC_WEIGHT *
                    heuristic(next_position, nx, ny),
                    -candidate, next_state))

        if end_state < 0:
            return None
        states = []
        state = end_state
        while state >= 0:
            heading, position = divmod(state, cell_count)
            y, x = divmod(position, width)
            states.append((x, y, heading))
            state = int(parents[state])
        states.reverse()
        return states

    @staticmethod
    def _footprint_waypoints(states):
        """Keep the end of each straight run; omit in-place rotation states."""
        moving = []
        for state in states:
            if not moving or state[:2] != moving[-1][:2]:
                moving.append(state)
        if len(moving) <= 1:
            return [moving[0][:2]] if moving else []
        output = [moving[0][:2]]
        heading = moving[1][2]
        for index in range(2, len(moving)):
            if moving[index][2] != heading:
                output.append(moving[index - 1][:2])
                heading = moving[index][2]
        if moving[-1][:2] != output[-1]:
            output.append(moving[-1][:2])
        return output

    def _a_star(self, start, goal, timeout=FOOTPRINT_ASTAR_TIMEOUT):
        """Plan the global centre line on the saved, radius-inflated map.

        The static PGM has already been inflated by the 0.23 m cylinder
        radius.  It should define global wall topology, not decide whether the
        robot can sweep its complete long body through its *current* heading.
        That immediate body collision is checked against live lidar by SCAN.
        """
        deadline = time.monotonic() + max(0.1, float(timeout))
        height, width = self.free.shape
        total = height * width
        start_i = start[1] * width + start[0]
        goal_i = goal[1] * width + goal[0]
        costs = np.full(total, np.inf, dtype=np.float64)
        parents = np.full(total, -1, dtype=np.int32)
        closed = np.zeros(total, dtype=np.bool_)
        costs[start_i] = 0.0
        queue = [(FOOTPRINT_HEURISTIC_WEIGHT *
                  math.hypot(goal[0] - start[0], goal[1] - start[1]),
                  0.0, start_i)]
        directions = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                      (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                      (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))
        expanded = 0
        while queue:
            _, cost, index = heapq.heappop(queue)
            if closed[index] or cost > costs[index] + 1e-9:
                continue
            if index == goal_i:
                break
            closed[index] = True
            expanded += 1
            if expanded % FOOTPRINT_ASTAR_YIELD_INTERVAL == 0:
                if time.monotonic() >= deadline:
                    raise FootprintSearchTimeout(
                        'global centre A* exceeded %.1fs after %d cells' %
                        (timeout, expanded))
                time.sleep(0)
            x, y = index % width, index // width
            for dx, dy, step in directions:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height) or not self.free[ny, nx]:
                    continue
                if dx and dy and (not self.free[y, nx] or not self.free[ny, x]):
                    continue
                ni = ny * width + nx
                # This remains a soft preference only.  The hard free/blocked
                # decision comes from the saved radius-inflated grid, while
                # current full-body clearance belongs to the live local map.
                near_wall_penalty = float(self.clearance_penalty[ny, nx])
                candidate = cost + step * (1.0 + near_wall_penalty)
                if candidate < costs[ni]:
                    costs[ni] = candidate
                    parents[ni] = index
                    estimate = candidate + FOOTPRINT_HEURISTIC_WEIGHT * math.hypot(
                        goal[0] - nx, goal[1] - ny)
                    heapq.heappush(queue, (estimate, candidate, ni))
        if parents[goal_i] < 0 and goal_i != start_i:
            return None
        cells = []
        index = goal_i
        while index >= 0:
            cells.append((index % width, index // width))
            if index == start_i:
                break
            index = int(parents[index])
        cells.reverse()
        return cells

    def _line_free(self, a, b):
        x0, y0 = a
        x1, y1 = b
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        error = dx - dy
        while True:
            if not self.free[y0, x0]:
                return False
            if x0 == x1 and y0 == y1:
                return True
            twice = 2 * error
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy

    def _simplify(self, cells):
        if len(cells) <= 2:
            return cells
        output = [cells[0]]
        index = 0
        while index < len(cells) - 1:
            farthest = min(len(cells) - 1, index + 30)
            while farthest > index + 1 and not self._line_free(cells[index], cells[farthest]):
                farthest -= 1
            output.append(cells[farthest])
            index = farthest
        return output

    @staticmethod
    def _densify(points, interval=0.15):
        if not points:
            return []
        dense = [(points[0][0], points[0][1], 0.05)]
        for a, b in zip(points, points[1:]):
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            count = max(1, int(math.ceil(distance / interval)))
            for i in range(1, count + 1):
                t = i / count
                dense.append((a[0] + (b[0] - a[0]) * t,
                              a[1] + (b[1] - a[1]) * t, 0.05))
        return dense

    def plan(self, goal_x, goal_y):
        with self.lock:
            if not self.enabled:
                return False, '请先切换到导航页面', None
            if not self.alignment_valid:
                return False, '请先完成本次开机后的地图位姿标定', None
            if not self.pose_t or time.time() - self.pose_t > 1.0:
                return False, '导航位姿离线，无法规划', None
            start_xy = (self.pose['x'], self.pose['y'])
            live_inflated = self.obstacle_inflated_planning.copy()
            live_age = (time.time() - self.obstacle_inflated_t
                        if self.obstacle_inflated_t else None)
        raw_start = self._to_cell(*start_xy)
        start = self._nearest_free(raw_start, max_distance=0.15)
        raw_goal = self._to_cell(float(goal_x), float(goal_y))
        temporary = None
        if start is None:
            if live_age is None or live_age > TEMP_START_LIVE_MAX_AGE_S:
                return False, '机器狗位于静态障碍内，但实时膨胀障碍层尚未就绪', None
            temporary = self._select_temporary_start(
                start_xy, raw_goal, live_inflated)
            if temporary is None:
                return False, (
                    '机器狗位于静态障碍内；%.1fm 内没有同时避开静态和实时障碍、'
                    '且可连接目标的临时起点' % TEMP_START_MAX_RADIUS_M), None
            start = temporary['cell']
            goal = temporary['goal']
        else:
            component = int(self.free_components[start[1], start[0]])
            goal = self._nearest_free(raw_goal, component=component)
        if goal is None:
            if self._nearest_free(raw_goal) is not None:
                return False, '目标与机器狗不在同一静态自由区域', None
            return False, '目标点不在可通行区域附近', None
        try:
            cells = self._a_star(start, goal)
        except FootprintSearchTimeout:
            return False, (
                '全局中心路径搜索超过%.0f秒，已停止本次搜索' %
                FOOTPRINT_ASTAR_TIMEOUT), None
        if not cells:
            return False, '静态自由区内没有连接该目标的路线', None
        waypoint_cells = self._simplify(cells)
        waypoints_xy = [self._to_world(cell) for cell in waypoint_cells]
        # The published reference starts at the selected static entry point;
        # SCAN connects the physical pose to it using live lidar. For Web
        # rendering only, include the current pose so that escape progress is
        # visible before the static global route begins.
        display_waypoints = ([start_xy] + waypoints_xy
                             if temporary is not None else waypoints_xy)
        dense = self._densify(display_waypoints)
        adjusted_goal = self._to_world(goal)
        with self.lock:
            self.global_waypoints = [(x, y, 0.0) for x, y in waypoints_xy]
            self.global_dense = dense
            self.global_progress = 0
            self.local = []
            self.local_waiting = False
            self.local_waiting_t = 0.0
            self.goal = {'x': round(adjusted_goal[0], 3), 'y': round(adjusted_goal[1], 3)}
            static_distance = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                  for a, b in zip(waypoints_xy, waypoints_xy[1:]))
            distance = static_distance
            if temporary is not None:
                distance += temporary['distance']
                self.temporary_start = {
                    'x': round(temporary['world'][0], 3),
                    'y': round(temporary['world'][1], 3),
                    'distance': round(temporary['distance'], 3),
                    'static_clearance': round(temporary['static_clearance'], 3),
                    'live_clearance': (None if not math.isfinite(
                        temporary['live_clearance']) else
                        round(temporary['live_clearance'], 3)),
                }
                self.route_message = (
                    '临时起点 (%.2f, %.2f)，先用实时局部规划驶出 %.2f m · '
                    '全局路径 %.1f m · %d 个关键点' % (
                        temporary['world'][0], temporary['world'][1],
                        temporary['distance'], static_distance,
                        len(waypoints_xy)))
            else:
                self.temporary_start = None
                self.route_message = '全局路径 %.1f m · %d 个关键点' % (
                    distance, len(waypoints_xy))
            transform = self.map_to_odom
            published = [(*self._map_to_odom_xy(x, y, transform), z)
                         for x, y, z in self.global_waypoints]
        adjusted = math.hypot(adjusted_goal[0] - goal_x, adjusted_goal[1] - goal_y) > 0.08
        message = self.route_message + ('（目标已吸附到可通行区）' if adjusted else '')
        return True, message, published

    def preview_plan(self, start_x, start_y, goal_x, goal_y):
        """Plan on the static grid without pose, ROS, SCAN, or live state."""
        values = (start_x, start_y, goal_x, goal_y)
        if not all(math.isfinite(float(value)) for value in values):
            return False, '假起点或目标坐标无效', None

        requested_start = (float(start_x), float(start_y))
        requested_goal = (float(goal_x), float(goal_y))
        start = self._nearest_free(self._to_cell(*requested_start))
        if start is None:
            return False, '假起点附近没有静态可通行区域', None
        raw_goal = self._to_cell(*requested_goal)
        component = int(self.free_components[start[1], start[0]])
        goal = self._nearest_free(raw_goal, component=component)
        if goal is None:
            if self._nearest_free(raw_goal) is not None:
                return False, '假起点与目标不在同一静态自由区域', None
            return False, '预演目标附近没有可通行区域', None
        try:
            cells = self._a_star(start, goal)
        except FootprintSearchTimeout:
            return False, (
                '离线全局中心路径搜索超过%.0f秒，已停止' %
                FOOTPRINT_ASTAR_TIMEOUT), None
        if not cells:
            return False, '假起点与目标在静态自由区内不可达', None

        waypoint_cells = self._simplify(cells)
        waypoints_xy = [self._to_world(cell) for cell in waypoint_cells]
        dense = self._densify(waypoints_xy)
        adjusted_start = self._to_world(start)
        adjusted_goal = self._to_world(goal)
        distance = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                       for a, b in zip(waypoints_xy, waypoints_xy[1:]))
        adjusted = []
        if math.hypot(adjusted_start[0] - requested_start[0],
                      adjusted_start[1] - requested_start[1]) > 0.08:
            adjusted.append('假起点')
        if math.hypot(adjusted_goal[0] - requested_goal[0],
                      adjusted_goal[1] - requested_goal[1]) > 0.08:
            adjusted.append('目标')
        suffix = '（%s已吸附到可通行区）' % '和'.join(adjusted) if adjusted else ''
        result = {
            'start': {'x': round(adjusted_start[0], 3),
                      'y': round(adjusted_start[1], 3)},
            'goal': {'x': round(adjusted_goal[0], 3),
                     'y': round(adjusted_goal[1], 3)},
            'path': _flat(dense),
            'distance': round(distance, 3),
            'keypoints': len(waypoints_xy),
        }
        return True, '离线路径 %.1f m · %d 个关键点%s' % (
            distance, len(waypoints_xy), suffix), result


class OperationManager:
    """Global mapping/navigation/map-library mutual exclusion and lifecycle."""

    def __init__(self, navigation, camera, save_callback, mapping_stream_callback,
                 reset_mapping_callback):
        self.lock = threading.Lock()
        self.navigation = navigation
        self.camera = camera
        self.save_callback = save_callback
        self.mapping_stream_callback = mapping_stream_callback
        self.reset_mapping_callback = reset_mapping_callback
        planner_running = self.planner_active(refresh=True)
        front_mapping = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet',
             'go2-front-pointlio.service'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        # A stale SCAN unit must never make a Web restart reinterpret the
        # front-lidar mapping backend as navigation odometry.
        if front_mapping and planner_running:
            subprocess.run(
                [SCAN_STOP], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, timeout=20.0)
            planner_running = False
        self.mode = 'navigation' if planner_running else 'mapping'
        self.mapping_active = False
        self._planner_active = self.mode == 'navigation'
        self._planner_checked = time.time()
        self.navigation.set_enabled(self.mode == 'navigation')
        if self.mode == 'navigation':
            self.mapping_stream_callback(False)
            self.camera.start()

    def planner_active(self, refresh=False):
        now = time.time()
        if not refresh and hasattr(self, '_planner_checked') and now - self._planner_checked < 1.0:
            return self._planner_active
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet', 'go2-scan-planner-dry.service'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._planner_active = result.returncode == 0
        self._planner_checked = now
        return self._planner_active

    @staticmethod
    def _script(path, timeout=20.0):
        result = subprocess.run([path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=timeout)
        return result.returncode == 0, result.stdout.strip()

    def ensure_lio_navigation_backend(self):
        """Navigation is allowed to consume only the XT16 LIO-SAM odometry."""
        front_active = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet',
             'go2-front-pointlio.service'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        lio_active = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet',
             'go2-lio-sam.service'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        xt16_active = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet', 'go2-xt16.service'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        if not front_active and lio_active and xt16_active:
            return True, '导航定位使用 XT16 + LIO-SAM'
        ok, output = self._script(LIO_NAV_RESTORE, timeout=120.0)
        if ok:
            return True, '导航定位后端已恢复为 XT16 + LIO-SAM'
        lines = [line for line in output.splitlines() if line.strip()]
        return False, (lines[-1] if lines else 'LIO-SAM 导航定位后端恢复失败')

    def ensure_navigation(self):
        backend_ok, backend_message = self.ensure_lio_navigation_backend()
        if not backend_ok:
            return False, backend_message or 'LIO-SAM 导航定位后端恢复失败'
        if self.planner_active(refresh=True):
            return True, '%s；SCAN-Planner 已运行' % backend_message
        ok, output = self._script(SCAN_START)
        self.planner_active(refresh=True)
        running = ok and self._planner_active
        return running, (('%s；SCAN-Planner 干跑已启动' % backend_message) if running else
                         (output or 'SCAN-Planner 启动失败'))

    def stop_planner(self):
        ok, output = self._script(SCAN_STOP)
        self._planner_active = False
        self._planner_checked = time.time()
        return ok, (output or ('SCAN-Planner 已停止' if ok else 'SCAN-Planner 停止失败'))

    def start_mapping(self):
        with self.lock:
            if self.mode != 'mapping':
                return False, '当前不在建图页面'
            if self.mapping_active:
                return True, '建图已经开启'
            self.mapping_stream_callback(True)
            self.mapping_active = True
            return True, '建图已开始'

    def stop_mapping(self, save_name=None, save=False):
        with self.lock:
            if not self.mapping_active:
                return True, '建图已经关闭'
            message = '建图已停止'
            if save:
                ok, message = self.save_callback(save_name)
                if not ok:
                    return False, message
            self.mapping_stream_callback(False)
            self.mapping_active = False
            return True, message

    def clear_mapping(self):
        """Discard only the current live LIO map, preserving saved map files."""
        with self.lock:
            if self.mode != 'mapping':
                return False, '当前不在建图页面'
            if not self.mapping_active:
                return False, '请先开始建图，再清除当前建图'

            # Resetting odometry invalidates every motion reference.  Close the
            # actuator gate even if global keyboard control was left enabled.
            self.navigation.set_chassis_enabled(False)
            # Unsubscribe before replacing the LIO process so a final message
            # from the old process cannot repopulate the cleared Web snapshot.
            self.mapping_stream_callback(False)
            try:
                ok, message = self.reset_mapping_callback()
            except Exception as exc:
                ok, message = False, '清除当前建图失败: %s' % exc
            finally:
                self.mapping_stream_callback(True)
            return ok, message

    def switch(self, target, save_name=None, preserve_teleop=False):
        if target not in ('mapping', 'navigation', 'maps'):
            return False, '未知模式', self.mode
        with self.lock:
            if target == self.mode:
                label = {'mapping': '建图', 'navigation': '导航', 'maps': '地图预览'}[target]
                if target == 'navigation':
                    backend_ok, backend_message = self.ensure_lio_navigation_backend()
                    if not backend_ok:
                        return False, ('导航定位后端校验失败: %s' %
                                       (backend_message or '未知错误')), self.mode
                    return True, '%s；已经在%s页面' % (backend_message, label), self.mode
                return True, '已经在%s页面' % label, self.mode

            previous = self.mode
            messages = []

            # 离开建图：先保存，再彻底取消大点云/雷达预览订阅。
            if previous == 'mapping' and self.mapping_active:
                if not save_name:
                    return False, '请输入地图名后再终止建图', self.mode
                saved, save_message = self.save_callback(save_name)
                if not saved:
                    return False, save_message, self.mode
                messages.append(save_message)
                self.mapping_stream_callback(False)
                self.mapping_active = False
                messages.append('建图点云流已关闭')

            # 离开导航：停止规划器、摄像头并清除活动路径。
            if previous == 'navigation':
                # Keyboard control is a global, page-independent command
                # source.  A page switch still tears down SCAN, camera and
                # route state, but must not disarm an actively owned teleop
                # session.  The WebSocket watchdog remains its safety owner.
                if not preserve_teleop:
                    self.navigation.set_chassis_enabled(False)
                stop_ok, stop_message = self._script(SCAN_STOP)
                self._planner_active = False
                self._planner_checked = time.time()
                self.camera.stop()
                self.navigation.set_enabled(False)
                self.navigation.clear_navigation('导航已停止，路径已清除')
                if not stop_ok:
                    return False, '导航停止异常: %s' % stop_message, self.mode
                messages.append(stop_message or '导航已停止，摄像头已关闭')
                if preserve_teleop:
                    messages.append('全局键盘控制保持启用')

            if target == 'navigation':
                backend_ok, backend_message = self.ensure_lio_navigation_backend()
                if not backend_ok:
                    return False, ('导航定位后端切换失败: %s' %
                                   (backend_message or '未知错误')), self.mode
                self.mode = 'navigation'
                self.navigation.set_enabled(True)
                self.mapping_stream_callback(False)
                messages.append(backend_message)
                messages.append('已进入导航页面；离线预演不会启动 SCAN 或摄像头')
                return True, '；'.join(filter(None, messages)), self.mode

            self.camera.stop()
            self.navigation.set_enabled(False)
            # 切回建图页也保持默认关闭，需用户显式开始。
            self.mapping_stream_callback(False)
            self.mapping_active = False
            self.mode = target
            if target == 'mapping':
                messages.append('已进入建图页面，建图功能保持关闭')
            else:
                messages.append('已进入离线地图预览，不订阅 ROS 大点云')
            return True, '；'.join(filter(None, messages)), self.mode

    def set_chassis_enabled(self, enabled, require_alignment=True):
        """Enable/lock the physical chassis gate without starting a route."""
        if self.mode != 'navigation':
            return False, '请先进入导航页面'
        return self.navigation.set_chassis_enabled(
            enabled, require_alignment=require_alignment)

    def shutdown(self):
        self.navigation.set_chassis_enabled(False)
        self.camera.stop()
