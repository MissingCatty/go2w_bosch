#!/usr/bin/env python3
"""Bounded holonomic recovery driven by the current lidar cloud.

SCAN remains the normal local planner.  This node is allowed to command a
short recovery primitive only while SCAN explicitly reports WAIT_REPLAN.  It
forward-simulates left/right strafe, reverse and in-place rotation only against
the latest deskewed cloud.  The global path biases progress direction but no
saved map, static cloud or static occupancy grid participates in collision
checking.  If no candidate is completely checked, it publishes zero and waits.
"""

from dataclasses import dataclass
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class Primitive:
    name: str
    vx: float
    vy: float
    wz: float
    duration: float
    bias: float = 0.0


@dataclass
class Evaluation:
    primitive: Primitive
    valid: bool
    score: float
    start_collisions: int
    end_collisions: int
    min_clearance: float
    end_clearance: float
    progress: float
    reason: str


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_matrix(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z))


def cloud_xyz(message):
    """Return XYZ without allocating Python tuples for every lidar point."""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float64)
    if any(fields[name].datatype != PointField.FLOAT32 or
           fields[name].count != 1 for name in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float64)
    count = int(message.width) * int(message.height)
    if count <= 0 or message.point_step <= 0:
        return np.empty((0, 3), dtype=np.float64)
    endian = '>' if message.is_bigendian else '<'
    values = []
    for name in ('x', 'y', 'z'):
        values.append(np.ndarray(
            shape=(count,), dtype=endian + 'f4', buffer=message.data,
            offset=int(fields[name].offset),
            strides=(int(message.point_step),)))
    points = np.column_stack(values).astype(np.float64, copy=False)
    return points[np.isfinite(points).all(axis=1)]


def simulate_primitive(start, primitive, step=0.08):
    """Integrate a constant body-frame twist into odom-frame SE(2) poses."""
    x, y, yaw = (float(value) for value in start)
    count = max(2, int(math.ceil(primitive.duration / step)) + 1)
    dt = primitive.duration / (count - 1)
    poses = [(x, y, yaw)]
    for _ in range(count - 1):
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * primitive.vx - s * primitive.vy) * dt
        y += (s * primitive.vx + c * primitive.vy) * dt
        yaw = normalize_angle(yaw + primitive.wz * dt)
        poses.append((x, y, yaw))
    return poses


def footprint_metrics(points_xy, pose, radius, offset):
    if len(points_xy) == 0:
        return 0, 10.0
    x, y, yaw = pose
    direction = np.asarray([math.cos(yaw), math.sin(yaw)])
    centre = np.asarray([x, y])
    front = centre + offset * direction
    rear = centre - offset * direction
    front_distance = np.linalg.norm(points_xy - front, axis=1)
    rear_distance = np.linalg.norm(points_xy - rear, axis=1)
    clearance = np.minimum(front_distance, rear_distance) - radius
    return int(np.count_nonzero(clearance <= 0.0)), float(clearance.min())


def evaluate_primitive(points_xy, start, primitive, radius, offset,
                       desired_direction):
    """Check the complete swept footprint, including safe overlap escape.

    A conservative footprint may already overlap at t=0.  Such a primitive is
    accepted only when overlap never increases, clearance never materially
    worsens, and it becomes fully clear before the end.  This permits moving
    away from a contact margin without granting a general collision exemption.
    """
    poses = simulate_primitive(start, primitive)
    metrics = [footprint_metrics(points_xy, pose, radius, offset)
               for pose in poses]
    start_count, start_clearance = metrics[0]
    became_clear = start_count == 0
    previous_count = start_count
    previous_clearance = start_clearance
    min_clearance = start_clearance
    clearance_sum = 0.0
    for index, (_pose, (collisions, clearance)) in enumerate(zip(poses, metrics)):
        min_clearance = min(min_clearance, clearance)
        clearance_sum += min(0.8, clearance)
        if start_count == 0:
            if collisions:
                return Evaluation(primitive, False, -math.inf, start_count,
                                  collisions, min_clearance, clearance, 0.0,
                                  'live footprint collision')
        else:
            if became_clear and collisions:
                return Evaluation(primitive, False, -math.inf, start_count,
                                  collisions, min_clearance, clearance, 0.0,
                                  're-entered live obstacle')
            if (collisions > previous_count or
                    clearance < previous_clearance - 0.025):
                return Evaluation(primitive, False, -math.inf, start_count,
                                  collisions, min_clearance, clearance, 0.0,
                                  'initial overlap worsens')
            if index > 0 and collisions == 0:
                became_clear = True
            previous_count = collisions
            previous_clearance = clearance

    end_count, end_clearance = metrics[-1]
    if end_count or not became_clear:
        return Evaluation(primitive, False, -math.inf, start_count, end_count,
                          min_clearance, end_clearance, 0.0,
                          'does not leave live obstacle')
    displacement = np.asarray([poses[-1][0] - start[0],
                               poses[-1][1] - start[1]])
    progress = float(displacement.dot(desired_direction))
    mean_clearance = clearance_sum / len(metrics)
    yaw_change = abs(normalize_angle(poses[-1][2] - start[2]))
    score = (2.5 * min(0.6, end_clearance) +
             1.0 * min(0.6, mean_clearance) +
             2.0 * progress - 0.25 * yaw_change + primitive.bias)
    return Evaluation(primitive, True, score, start_count, end_count,
                      min_clearance, end_clearance, progress, 'safe')


class RealtimeRecovery(Node):
    def __init__(self):
        super().__init__('go2_realtime_recovery')
        self.radius = float(self.declare_parameter(
            'double_cylinder_radius', 0.23).value)
        self.offset = float(self.declare_parameter(
            'double_cylinder_offset', 0.12).value)
        self.obstacle_min_below_body = float(self.declare_parameter(
            'obstacle_min_below_body', 0.39).value)
        self.obstacle_max_above_body = float(self.declare_parameter(
            'obstacle_max_above_body', 0.50).value)
        self.activation_clearance = float(self.declare_parameter(
            'activation_clearance', 0.45).value)
        self.activation_delay = float(self.declare_parameter(
            'activation_delay', 1.0).value)
        self.cloud_topic = str(self.declare_parameter(
            'cloud_topic', '/scan_planner/local_cloud').value)
        self.cloud_timeout = float(self.declare_parameter(
            'cloud_timeout', 0.35).value)
        self.pose_timeout = float(self.declare_parameter(
            'pose_timeout', 0.40).value)
        self.cooldown = float(self.declare_parameter(
            'cooldown', 1.5).value)
        self.max_attempts = int(self.declare_parameter(
            'max_attempts', 3).value)
        strafe_speed = abs(float(self.declare_parameter(
            'strafe_speed', 0.10).value))
        reverse_speed = abs(float(self.declare_parameter(
            'reverse_speed', 0.10).value))
        rotate_speed = abs(float(self.declare_parameter(
            'rotate_speed', 0.30).value))
        translate_duration = float(self.declare_parameter(
            'translate_duration', 1.8).value)
        rotate_duration = float(self.declare_parameter(
            'rotate_duration', 1.4).value)
        self.primitives = [
            Primitive('左侧移', 0.0, strafe_speed, 0.0,
                      translate_duration, 0.08),
            Primitive('右侧移', 0.0, -strafe_speed, 0.0,
                      translate_duration, 0.08),
            Primitive('左后侧移', -0.7 * reverse_speed,
                      0.7 * strafe_speed, 0.0, translate_duration, 0.02),
            Primitive('右后侧移', -0.7 * reverse_speed,
                      -0.7 * strafe_speed, 0.0, translate_duration, 0.02),
            Primitive('后退', -reverse_speed, 0.0, 0.0,
                      translate_duration, -0.02),
            Primitive('原地左转', 0.0, 0.0, rotate_speed,
                      rotate_duration, -0.08),
            Primitive('原地右转', 0.0, 0.0, -rotate_speed,
                      rotate_duration, -0.08),
        ]

        self.body_pose = None
        self.body_pose_t = None
        self.sensor_pose = None
        self.cloud_points = np.empty((0, 3), dtype=np.float64)
        self.cloud_t = None
        self.cloud_input_t = None
        self._cloud_sub = None
        self.global_path = []
        self.local_waiting = False
        self.waiting_since = None
        self.cancelled = True
        self.active = False
        self.active_primitive = None
        self.active_started = None
        self.active_start_pose = None
        self.cooldown_until = 0.0
        self.attempts = 0
        self.last_reason = '等待导航'

        self.command_pub = self.create_publisher(
            Twist, '/scan_planner/recovery_cmd', 10)
        self.active_pub = self.create_publisher(
            Bool, '/scan_planner/recovery_active', 10)
        self.status_pub = self.create_publisher(
            String, '/scan_planner/recovery_status', 10)
        self.create_subscription(
            Odometry, '/scan_planner/body_pose', self.on_body_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/scan_planner/sensor_pose', self.on_sensor_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            Path, '/scan_planner/global_path', self.on_path, 10)
        self.create_subscription(
            Bool, '/scan_planner/planning/local_waiting',
            self.on_local_waiting, 10)
        self.create_subscription(
            Bool, '/scan_planner/cancel', self.on_cancel, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.25, self.publish_status)
        self.get_logger().info(
            'Realtime recovery ready: %d bounded holonomic primitives, '
            'starts only in WAIT_REPLAN' % len(self.primitives))

    def on_body_pose(self, message):
        pose = message.pose.pose
        self.body_pose = (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z), quaternion_yaw(pose.orientation))
        self.body_pose_t = time.monotonic()

    def on_sensor_pose(self, message):
        self.sensor_pose = message.pose.pose

    def set_cloud_subscription(self, enabled):
        if enabled and self._cloud_sub is None:
            self._cloud_sub = self.create_subscription(
                PointCloud2, self.cloud_topic, self.on_cloud,
                qos_profile_sensor_data)
        elif not enabled and self._cloud_sub is not None:
            self.destroy_subscription(self._cloud_sub)
            self._cloud_sub = None
            self.cloud_points = np.empty((0, 3), dtype=np.float64)
            self.cloud_t = None

    def on_cloud(self, message):
        self.cloud_input_t = time.monotonic()
        # Current-cloud collision work is needed only during recovery.  Wait
        # for a new frame after WAIT_REPLAN rather than burning a CPU core on
        # every 22k-point scan throughout ordinary navigation or cancellation.
        if self.cancelled or not self.local_waiting:
            return
        if self.sensor_pose is None:
            return
        sensor_points = cloud_xyz(message)
        if len(sensor_points) == 0:
            return
        rotation = quaternion_matrix(self.sensor_pose.orientation)
        translation = np.asarray([
            self.sensor_pose.position.x,
            self.sensor_pose.position.y,
            self.sensor_pose.position.z,
        ], dtype=np.float64)
        world = sensor_points @ rotation.T + translation
        if self.body_pose is not None:
            body_z = self.body_pose[2]
            mask = ((world[:, 2] >= body_z - self.obstacle_min_below_body) &
                    (world[:, 2] <= body_z + self.obstacle_max_above_body))
            world = world[mask]
        if len(world):
            # One point per 4 cm voxel is sufficient for footprint collision
            # checks and keeps the 20 Hz recovery loop inexpensive.
            keys = np.floor(world / 0.04).astype(np.int32)
            _, unique = np.unique(keys, axis=0, return_index=True)
            world = world[np.sort(unique)]
        self.cloud_points = world
        self.cloud_t = time.monotonic()

    def on_path(self, message):
        self.global_path = [
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in message.poses]

    def on_local_waiting(self, message):
        waiting = bool(message.data)
        if waiting and not self.local_waiting:
            self.waiting_since = time.monotonic()
            self.attempts = 0
            self.cooldown_until = 0.0
            self.cloud_points = np.empty((0, 3), dtype=np.float64)
            self.cloud_t = None
        self.set_cloud_subscription(waiting and not self.cancelled)
        if not waiting:
            self.stop('SCAN 已恢复局部轨迹')
            self.waiting_since = None
            self.attempts = 0
        self.local_waiting = waiting

    def on_cancel(self, message):
        self.cancelled = bool(message.data)
        self.set_cloud_subscription(self.local_waiting and not self.cancelled)
        if self.cancelled:
            self.stop('导航已取消')

    def desired_direction(self):
        pose = self.body_pose
        if pose is None:
            return np.asarray([1.0, 0.0])
        fallback = np.asarray([math.cos(pose[3]), math.sin(pose[3])])
        if len(self.global_path) < 2:
            return fallback
        points = np.asarray(self.global_path, dtype=np.float64)
        current = np.asarray(pose[:2])
        nearest = int(np.argmin(np.linalg.norm(points - current, axis=1)))
        target = nearest + 1
        while (target < len(points) and
               np.linalg.norm(points[target] - current) < 0.5):
            target += 1
        if target >= len(points):
            target = len(points) - 1
        direction = points[target] - current
        norm = np.linalg.norm(direction)
        return fallback if norm < 1e-6 else direction / norm

    def obstacle_points_xy(self):
        if self.body_pose is None or len(self.cloud_points) == 0:
            return np.empty((0, 2), dtype=np.float64)
        current = np.asarray(self.body_pose[:2])
        delta = self.cloud_points[:, :2] - current
        nearby = np.einsum('ij,ij->i', delta, delta) <= 2.5 ** 2
        return self.cloud_points[nearby, :2]

    def evaluate(self, primitive, duration=None):
        if duration is not None:
            primitive = Primitive(
                primitive.name, primitive.vx, primitive.vy, primitive.wz,
                max(0.15, duration), primitive.bias)
        start = (self.body_pose[0], self.body_pose[1], self.body_pose[3])
        return evaluate_primitive(
            self.obstacle_points_xy(), start, primitive,
            self.radius, self.offset, self.desired_direction())

    def select_primitive(self):
        evaluations = [self.evaluate(item) for item in self.primitives]
        valid = [item for item in evaluations if item.valid]
        summary = ', '.join(
            '%s:%s' % (item.primitive.name,
                       ('%.2f' % item.score) if item.valid else item.reason)
            for item in evaluations)
        self.get_logger().info('Recovery candidates: %s' % summary)
        return max(valid, key=lambda item: item.score) if valid else None

    def publish_command(self, primitive):
        message = Twist()
        message.linear.x = primitive.vx
        message.linear.y = primitive.vy
        message.angular.z = primitive.wz
        self.command_pub.publish(message)
        self.active_pub.publish(Bool(data=True))

    def stop(self, reason, cooldown=True):
        was_active = self.active
        self.active = False
        self.active_primitive = None
        self.active_started = None
        self.active_start_pose = None
        self.command_pub.publish(Twist())
        self.active_pub.publish(Bool(data=False))
        self.last_reason = reason
        if was_active and cooldown:
            self.cooldown_until = time.monotonic() + self.cooldown
            self.get_logger().info(reason)

    def tick(self):
        now = time.monotonic()
        if self.cancelled or not self.local_waiting:
            if self.active:
                self.stop('恢复条件已撤销')
            return
        if (self.body_pose is None or self.body_pose_t is None or
                now - self.body_pose_t > self.pose_timeout):
            self.stop('恢复保持零速：本体位姿超时')
            return
        if (self.cloud_t is None or now - self.cloud_t > self.cloud_timeout):
            self.stop('恢复保持零速：实时点云超时')
            return
        if self.active:
            elapsed = now - self.active_started
            remaining = self.active_primitive.duration - elapsed
            if remaining <= 0.0:
                name = self.active_primitive.name
                self.stop('%s完成，交回 SCAN 重规划' % name)
                return
            # Revalidate the remaining swept body on every fresh cloud.  A
            # person stepping into the chosen side immediately aborts motion.
            evaluation = self.evaluate(
                self.active_primitive, min(0.6, remaining))
            if not evaluation.valid:
                name = self.active_primitive.name
                self.stop('%s中止：%s' % (name, evaluation.reason))
                return
            self.publish_command(self.active_primitive)
            return

        if self.waiting_since is None or now - self.waiting_since < self.activation_delay:
            self.last_reason = '等待 SCAN 自主重规划'
            return
        if now < self.cooldown_until:
            self.last_reason = '脱困动作后等待 SCAN 重规划'
            return
        if self.attempts >= self.max_attempts:
            self.last_reason = '实时脱困已达次数上限，保持零速等待环境变化'
            return

        points_xy = self.obstacle_points_xy()
        _, clearance = footprint_metrics(
            points_xy,
            (self.body_pose[0], self.body_pose[1], self.body_pose[3]),
            self.radius, self.offset)
        if clearance > self.activation_clearance:
            # SCAN may still be looking at a cached hit.  Do not move when the
            # current cloud does not corroborate a nearby obstacle.
            self.last_reason = '当前点云近身区域清晰，等待 SCAN 清除旧占据'
            return

        selected = self.select_primitive()
        if selected is None:
            self.last_reason = '实时点云下无安全脱困动作，保持零速'
            return
        self.attempts += 1
        self.active = True
        self.active_primitive = selected.primitive
        self.active_started = now
        self.active_start_pose = self.body_pose
        self.last_reason = '执行%s（第%d/%d次）' % (
            selected.primitive.name, self.attempts, self.max_attempts)
        self.get_logger().warning(
            '%s: score=%.2f, clearance %.2f -> %.2f m, progress=%.2f m' %
            (self.last_reason, selected.score, selected.min_clearance,
             selected.end_clearance, selected.progress))
        self.publish_command(selected.primitive)

    def publish_status(self):
        status = {
            'active': self.active,
            'action': (None if self.active_primitive is None else
                       self.active_primitive.name),
            'attempts': self.attempts,
            'max_attempts': self.max_attempts,
            'local_waiting': self.local_waiting,
            'cancelled': self.cancelled,
            'cloud_age': (None if self.cloud_t is None else
                          round(time.monotonic() - self.cloud_t, 3)),
            'cloud_input_age': (None if self.cloud_input_t is None else
                                round(time.monotonic() - self.cloud_input_t, 3)),
            'reason': self.last_reason,
        }
        self.status_pub.publish(String(
            data=json.dumps(status, ensure_ascii=False, separators=(',', ':'))))

    def shutdown(self):
        for _ in range(3):
            self.stop('恢复节点退出', cooldown=False)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
