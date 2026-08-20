#!/usr/bin/env python3
"""Publish a processed GO2-W navigation map and SCAN static obstacle layer."""

import json
import hashlib
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def read_pgm(path):
    with open(path, 'rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('only binary P5 PGM maps are supported')

        def tokens(count):
            result = []
            while len(result) < count:
                line = stream.readline()
                if not line:
                    raise ValueError('truncated PGM header')
                line = line.split(b'#', 1)[0]
                result.extend(line.split())
            return result

        width, height = (int(value) for value in tokens(2))
        maximum = int(tokens(1)[0])
        if maximum != 255:
            raise ValueError('PGM maximum must be 255')
        pixels = np.frombuffer(stream.read(), dtype=np.uint8)
    if len(pixels) != width * height:
        raise ValueError('PGM payload size does not match its dimensions')
    return pixels.reshape(height, width)


def read_ascii_pcd(path):
    with open(path, 'r', encoding='ascii') as stream:
        data_mode = None
        rows = []
        for line in stream:
            stripped = line.strip()
            if data_mode is None:
                if stripped.upper().startswith('DATA '):
                    data_mode = stripped.split(None, 1)[1].lower()
                    if data_mode != 'ascii':
                        raise ValueError('only ASCII PCD files are supported')
                continue
            if stripped:
                values = stripped.split()
                rows.append((float(values[0]), float(values[1]), float(values[2])))
    if data_mode is None:
        raise ValueError('PCD DATA header is missing')
    return np.asarray(rows, dtype=np.float32).reshape(-1, 3)


def boot_id():
    try:
        with open('/proc/sys/kernel/random/boot_id', 'r', encoding='ascii') as stream:
            return stream.read().strip()
    except OSError:
        return ''


def map_signature(metadata):
    relevant = dict(metadata)
    relevant['source'] = os.path.realpath(str(metadata.get('source', '')))
    encoded = json.dumps(
        relevant, sort_keys=True, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


class StaticNavigationMap(Node):
    def __init__(self):
        super().__init__('go2_static_navigation_map')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('inflated_map_yaml', '')
        self.declare_parameter('obstacle_pcd', '')
        self.declare_parameter('metadata_json', '')
        self.declare_parameter('alignment_json', '')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_z', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        self.declare_parameter('publish_period', 5.0)

        map_yaml = str(self.get_parameter('map_yaml').value)
        inflated_map_yaml = str(self.get_parameter('inflated_map_yaml').value)
        obstacle_pcd = str(self.get_parameter('obstacle_pcd').value)
        metadata_json = str(self.get_parameter('metadata_json').value)
        alignment_json = str(self.get_parameter('alignment_json').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.tx = float(self.get_parameter('map_to_odom_x').value)
        self.ty = float(self.get_parameter('map_to_odom_y').value)
        self.tz = float(self.get_parameter('map_to_odom_z').value)
        self.yaw = float(self.get_parameter('map_to_odom_yaw').value)

        preparation = None
        if metadata_json and os.path.isfile(metadata_json):
            with open(metadata_json, 'r', encoding='utf-8') as stream:
                preparation = json.load(stream)
        self.alignment_valid = False
        if alignment_json:
            try:
                with open(alignment_json, 'r', encoding='utf-8') as stream:
                    alignment = json.load(stream)
                if alignment.get('boot_id') != boot_id():
                    raise RuntimeError('alignment belongs to a previous boot')
                if (preparation is None or
                        alignment.get('map_signature') != map_signature(preparation)):
                    raise RuntimeError('alignment does not match the active map')
                if os.path.realpath(str(alignment.get('map_source', ''))) != os.path.realpath(
                        str(preparation.get('source', ''))):
                    raise RuntimeError('alignment map source mismatch')
                transform = alignment.get('map_to_odom')
                if (not isinstance(transform, list) or len(transform) != 4 or
                        not all(math.isfinite(float(value)) for value in transform)):
                    raise RuntimeError('alignment transform is invalid')
                self.tx, self.ty, self.tz, self.yaw = (float(value) for value in transform)
                self.alignment_valid = True
            except (OSError, ValueError, TypeError, RuntimeError) as exc:
                # Keep the pose adapter available so Web can collect the live
                # odometry needed for calibration.  Goal submission and the
                # physical chassis remain independently blocked while invalid.
                self.tx = self.ty = self.tz = self.yaw = 0.0
                self.get_logger().warning(
                    'navigation alignment unavailable; calibration only: %s' % exc)

        if not map_yaml or not os.path.isfile(map_yaml):
            raise FileNotFoundError('navigation map YAML not found: %s' % map_yaml)
        if not inflated_map_yaml or not os.path.isfile(inflated_map_yaml):
            raise FileNotFoundError(
                'inflated navigation map YAML not found: %s' % inflated_map_yaml)
        if not obstacle_pcd or not os.path.isfile(obstacle_pcd):
            raise FileNotFoundError('navigation obstacle PCD not found: %s' % obstacle_pcd)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/navigation/map', qos)
        # Keep the footprint-inflated grid separate from the live voxel map.
        # SCAN treats it as an immutable wall/free-space boundary while the
        # lidar layer remains responsible for chairs, people and other
        # temporary obstacles.
        self.inflated_map_pub = self.create_publisher(
            OccupancyGrid, '/navigation/inflated_map', qos)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/scan_planner/static_obstacles', qos)
        self.metadata_pub = self.create_publisher(String, '/navigation/map_metadata', qos)
        self.map_service = self.create_service(GetMap, '/navigation/static_map', self.on_get_map)

        self.map_msg = self.load_map(map_yaml)
        self.inflated_map_msg = self.load_map(inflated_map_yaml)
        self.cloud_msg = self.load_cloud(obstacle_pcd)
        metadata = {
            'map_yaml': map_yaml,
            'inflated_map_yaml': inflated_map_yaml,
            'obstacle_pcd': obstacle_pcd,
            'frame_id': self.frame_id,
            'map_to_odom': [self.tx, self.ty, self.tz, self.yaw],
            'alignment_valid': self.alignment_valid,
            'occupancy_cells': len(self.map_msg.data),
            'obstacle_points': self.cloud_msg.width,
        }
        if preparation is not None:
            metadata['preparation'] = preparation
        metadata['alignment_json'] = alignment_json
        self.metadata_msg = String(data=json.dumps(metadata, ensure_ascii=False))

        period = max(0.5, float(self.get_parameter('publish_period').value))
        self.timer = self.create_timer(period, self.publish)
        self.initial_timer = self.create_timer(0.2, self.publish_initial)
        self.initial_sent = False
        self.get_logger().info(
            'static navigation map loaded: %dx%d, %d obstacle points, frame=%s' % (
                self.map_msg.info.width, self.map_msg.info.height,
                self.cloud_msg.width, self.frame_id))

    def transform_xy(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * x - s * y + self.tx, s * x + c * y + self.ty

    def load_map(self, yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        image_path = str(config['image'])
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        pixels = read_pgm(image_path)
        occupied_threshold = float(config.get('occupied_thresh', 0.65))
        free_threshold = float(config.get('free_thresh', 0.196))
        negate = bool(config.get('negate', 0))
        occupancy_probability = pixels.astype(np.float32) / 255.0
        if not negate:
            occupancy_probability = 1.0 - occupancy_probability
        values = np.full(pixels.shape, -1, dtype=np.int8)
        values[occupancy_probability > occupied_threshold] = 100
        values[occupancy_probability < free_threshold] = 0
        # Image rows run top-to-bottom; OccupancyGrid rows run min-y to max-y.
        values = values[::-1]

        origin = config.get('origin', [0.0, 0.0, 0.0])
        ox, oy = self.transform_xy(float(origin[0]), float(origin[1]))
        origin_yaw = float(origin[2]) + self.yaw
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(config['resolution'])
        msg.info.width = int(values.shape[1])
        msg.info.height = int(values.shape[0])
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.position.z = self.tz
        msg.info.origin.orientation = Quaternion(
            z=math.sin(origin_yaw / 2.0), w=math.cos(origin_yaw / 2.0))
        msg.data = values.ravel().tolist()
        return msg

    def load_cloud(self, pcd_path):
        points = read_ascii_pcd(pcd_path)
        if len(points):
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            x = points[:, 0].copy()
            y = points[:, 1].copy()
            points[:, 0] = c * x - s * y + self.tx
            points[:, 1] = s * x + c * y + self.ty
            points[:, 2] += self.tz
        points = np.ascontiguousarray(points.astype('<f4', copy=False))
        msg = PointCloud2()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        msg.is_dense = True
        return msg

    def stamp_messages(self):
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.map_msg.info.map_load_time = stamp
        self.inflated_map_msg.header.stamp = stamp
        self.inflated_map_msg.info.map_load_time = stamp
        self.cloud_msg.header.stamp = stamp

    def publish(self):
        self.stamp_messages()
        self.map_pub.publish(self.map_msg)
        self.inflated_map_pub.publish(self.inflated_map_msg)
        self.cloud_pub.publish(self.cloud_msg)
        self.metadata_pub.publish(self.metadata_msg)

    def publish_initial(self):
        if self.initial_sent:
            return
        self.initial_sent = True
        self.publish()
        self.initial_timer.cancel()

    def on_get_map(self, request, response):
        del request
        self.stamp_messages()
        response.map = self.map_msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StaticNavigationMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
