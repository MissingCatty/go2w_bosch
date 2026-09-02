#!/usr/bin/env python3
"""Fuse LIO-SAM's deskewed cloud with a self-filtered XT16 near field.

LIO-SAM deliberately drops returns closer than ``lidarMinRange`` (currently
1.0 m).  That is reasonable for odometry but leaves a dangerous blind annulus
for local collision avoidance.  This node keeps the deskewed cloud unchanged
and adds only close raw XT16 returns after transforming them into the body
frame for height and self-body filtering.  The published points remain in the
lidar frame expected by SCAN's existing sensor-pose transform.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def cloud_xyz(message):
    """Return a compact float32 N×3 view/copy from an arbitrary PointCloud2."""
    offsets = {field.name: field.offset for field in message.fields}
    if not all(axis in offsets for axis in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4'] * 3,
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': int(message.point_step),
    })
    count = int(message.width * message.height)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float32)
    cloud = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((cloud['x'], cloud['y'], cloud['z'])).astype(
        np.float32, copy=False)
    return points[np.isfinite(points).all(axis=1)]


def voxel_sample(points, resolution):
    if len(points) == 0 or resolution <= 0.0:
        return points
    # np.unique(..., axis=0) creates a structured array and is surprisingly
    # expensive at lidar rate. Pack the three bounded voxel coordinates into
    # one int64 key so the same first-point policy uses the fast 1-D path.
    coordinates = np.floor(points / resolution).astype(np.int64)
    coordinates -= coordinates.min(axis=0)
    spans = coordinates.max(axis=0) + 1
    keys = ((coordinates[:, 0] * spans[1] + coordinates[:, 1]) *
            spans[2] + coordinates[:, 2])
    _, indices = np.unique(keys, return_index=True)
    return points[np.sort(indices)]


class NearFieldCloudFuser(Node):

    def __init__(self):
        super().__init__('go2_near_field_cloud_fuser')
        self.far_topic = str(self.declare_parameter(
            'far_topic', '/lio_sam/deskew/cloud_deskewed').value)
        self.raw_topic = str(self.declare_parameter(
            'raw_topic', '/unitree/slam_lidar/points').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/scan_planner/local_cloud').value)
        self.near_min = float(self.declare_parameter('near_min_range', 0.25).value)
        self.near_max = float(self.declare_parameter('near_max_range', 1.05).value)
        self.near_voxel = float(self.declare_parameter('near_voxel_size', 0.04).value)
        self.max_raw_age = float(self.declare_parameter('max_raw_age', 0.25).value)

        self.base_to_lidar_x = float(self.declare_parameter(
            'base_to_lidar_x', 0.1701).value)
        self.base_to_lidar_y = float(self.declare_parameter(
            'base_to_lidar_y', 0.0).value)
        self.base_to_lidar_z = float(self.declare_parameter(
            'base_to_lidar_z', 0.0908).value)
        yaw = float(self.declare_parameter(
            'base_to_lidar_yaw', math.pi / 2.0).value)
        self.mount_cos = math.cos(yaw)
        self.mount_sin = math.sin(yaw)

        self.body_radius = float(self.declare_parameter(
            'self_filter_radius', 0.24).value)
        self.body_offset = float(self.declare_parameter(
            'self_filter_offset', 0.12).value)
        self.self_z_min = float(self.declare_parameter(
            'self_filter_z_min', -0.55).value)
        self.self_z_max = float(self.declare_parameter(
            'self_filter_z_max', 0.30).value)
        self.obstacle_z_min = float(self.declare_parameter(
            'obstacle_body_z_min', -0.39).value)
        self.obstacle_z_max = float(self.declare_parameter(
            'obstacle_body_z_max', 0.55).value)

        if not (0.0 <= self.near_min < self.near_max):
            raise ValueError('near_min_range must be smaller than near_max_range')
        if self.body_radius <= 0.0 or self.body_offset < 0.0:
            raise ValueError('self-filter footprint parameters are invalid')

        self.raw_message = None
        self.raw_received_time = None
        self.last_processed_raw = None
        self.near_points = np.empty((0, 3), dtype=np.float32)
        self.near_frame = ''
        self.near_time = None
        self.raw_candidate_count = 0
        self.self_filtered_count = 0
        self.height_filtered_count = 0
        self.last_log = 0.0

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.raw_topic, self.on_raw, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.far_topic, self.on_far, qos_profile_sensor_data)
        self.get_logger().info(
            'Near-field fusion ready: %.2f..%.2f m raw XT16 + LIO deskew -> %s' % (
                self.near_min, self.near_max, self.output_topic))

    def on_raw(self, message):
        # The fused output is clocked by the deskewed LIO cloud. Keep only the
        # newest raw frame here, then filter it once when the next far frame
        # arrives. Processing every intermediate raw frame wastes CPU without
        # producing an additional fused observation.
        self.raw_message = message
        self.raw_received_time = time.monotonic()

    def update_near(self, message, received_time):
        points = cloud_xyz(message)
        if len(points) == 0:
            self.near_points = np.empty((0, 3), dtype=np.float32)
            self.near_frame = message.header.frame_id
            self.near_time = received_time
            return

        ranges = np.linalg.norm(points, axis=1)
        candidate_mask = (ranges >= self.near_min) & (ranges <= self.near_max)
        candidates = points[candidate_mask]
        self.raw_candidate_count = int(len(candidates))
        if len(candidates) == 0:
            self.near_points = np.empty((0, 3), dtype=np.float32)
            self.near_frame = message.header.frame_id
            self.near_time = received_time
            return

        # Lidar frame -> body frame, used only to decide what to retain.
        body_x = (self.mount_cos * candidates[:, 0] -
                  self.mount_sin * candidates[:, 1] + self.base_to_lidar_x)
        body_y = (self.mount_sin * candidates[:, 0] +
                  self.mount_cos * candidates[:, 1] + self.base_to_lidar_y)
        body_z = candidates[:, 2] + self.base_to_lidar_z

        front_distance2 = (body_x - self.body_offset) ** 2 + body_y ** 2
        rear_distance2 = (body_x + self.body_offset) ** 2 + body_y ** 2
        within_capsule = np.minimum(front_distance2, rear_distance2) <= (
            self.body_radius ** 2)
        within_self_height = ((body_z >= self.self_z_min) &
                              (body_z <= self.self_z_max))
        self_mask = within_capsule & within_self_height
        height_mask = ((body_z >= self.obstacle_z_min) &
                       (body_z <= self.obstacle_z_max))
        keep = height_mask & ~self_mask
        self.self_filtered_count = int(self_mask.sum())
        self.height_filtered_count = int((~height_mask & ~self_mask).sum())
        self.near_points = voxel_sample(candidates[keep], self.near_voxel)
        self.near_frame = message.header.frame_id
        self.near_time = received_time

    def on_far(self, message):
        far_points = cloud_xyz(message)
        if len(far_points) == 0:
            return
        far_input_count = len(far_points)
        far_keep = ((far_points[:, 2] + self.base_to_lidar_z) >=
                    self.obstacle_z_min)
        far_ground_filtered = int((~far_keep).sum())
        far_points = far_points[far_keep]
        now = time.monotonic()
        if (self.raw_message is not None and
                self.raw_message is not self.last_processed_raw):
            raw_message = self.raw_message
            raw_received_time = self.raw_received_time
            self.update_near(raw_message, raw_received_time)
            self.last_processed_raw = raw_message
        near_fresh = (self.near_time is not None and
                      now - self.near_time <= self.max_raw_age and
                      self.near_frame == message.header.frame_id)
        if near_fresh and len(self.near_points):
            fused = np.concatenate((far_points, self.near_points), axis=0)
        else:
            fused = far_points

        fused = np.ascontiguousarray(fused, dtype='<f4')
        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = int(len(fused))
        output.fields = XYZ_FIELDS
        output.is_bigendian = False
        output.point_step = 12
        output.row_step = output.point_step * output.width
        output.is_dense = bool(np.isfinite(fused).all())
        output.data = fused.tobytes()
        self.publisher.publish(output)

        if now - self.last_log >= 5.0:
            self.last_log = now
            self.get_logger().info(
                'local_cloud: far=%d/%d ground=%d near=%d/%d self=%d height=%d fresh=%s fused=%d' % (
                    len(far_points), far_input_count, far_ground_filtered,
                    len(self.near_points), self.raw_candidate_count,
                    self.self_filtered_count, self.height_filtered_count,
                    near_fresh, len(fused)))


def main(args=None):
    rclpy.init(args=args)
    node = NearFieldCloudFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
