#!/usr/bin/env python3
"""Convert LIO-SAM lidar odometry into SCAN-Planner sensor and body poses."""

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # Unit-quaternion rotation, expanded to avoid another dependency.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def normalized(q):
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    return tuple(value / norm for value in q)


class LioPoseAdapter(Node):
    def __init__(self):
        super().__init__('go2_scan_lio_pose_adapter')
        self.declare_parameter('source_topic', '/lio_sam/mapping/odometry')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('sensor_frame_id', 'rslidar')
        self.declare_parameter('base_to_lidar_x', 0.1701)
        self.declare_parameter('base_to_lidar_y', 0.0)
        self.declare_parameter('base_to_lidar_z', 0.0908)
        self.declare_parameter('base_to_lidar_yaw', math.pi / 2.0)
        self.declare_parameter('world_z_offset', 0.53)
        self.declare_parameter('velocity_alpha', 0.35)
        self.declare_parameter('max_velocity_sample', 2.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.sensor_frame_id = self.get_parameter('sensor_frame_id').value
        self.translation = tuple(
            float(self.get_parameter(name).value)
            for name in ('base_to_lidar_x', 'base_to_lidar_y', 'base_to_lidar_z')
        )
        yaw = float(self.get_parameter('base_to_lidar_yaw').value)
        self.q_base_lidar_inv = (0.0, 0.0, -math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        self.z_offset = float(self.get_parameter('world_z_offset').value)
        self.velocity_alpha = float(self.get_parameter('velocity_alpha').value)
        self.max_velocity_sample = float(self.get_parameter('max_velocity_sample').value)

        self.body_pub = self.create_publisher(Odometry, '/scan_planner/body_pose', qos_profile_sensor_data)
        self.sensor_pub = self.create_publisher(Odometry, '/scan_planner/sensor_pose', qos_profile_sensor_data)
        source = str(self.get_parameter('source_topic').value)
        self.sub = self.create_subscription(Odometry, source, self.on_odom, qos_profile_sensor_data)
        self.last_stamp = None
        self.last_position = None
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.count = 0
        self.get_logger().info(
            'LIO pose adapter: %s -> sensor/body poses, z offset %.3fm' % (source, self.z_offset))

    def on_odom(self, msg):
        q_lidar = normalized((
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        if q_lidar is None:
            return

        sensor = Odometry()
        sensor.header = msg.header
        sensor.header.frame_id = self.frame_id
        sensor.child_frame_id = self.sensor_frame_id
        sensor.pose = copy.deepcopy(msg.pose)
        sensor.twist = copy.deepcopy(msg.twist)
        sensor.pose.pose.position.z += self.z_offset
        self.sensor_pub.publish(sensor)

        q_body = normalized(quat_multiply(q_lidar, self.q_base_lidar_inv))
        if q_body is None:
            return
        rotated_translation = quat_rotate(q_body, self.translation)
        lidar_position = (
            sensor.pose.pose.position.x,
            sensor.pose.pose.position.y,
            sensor.pose.pose.position.z,
        )
        body_position = tuple(
            lidar_position[i] - rotated_translation[i] for i in range(3))

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if 0.02 <= dt <= 1.0:
                sample = [(body_position[i] - self.last_position[i]) / dt for i in range(3)]
                if math.sqrt(sum(value * value for value in sample)) <= self.max_velocity_sample:
                    a = min(1.0, max(0.0, self.velocity_alpha))
                    self.filtered_velocity = [
                        a * sample[i] + (1.0 - a) * self.filtered_velocity[i]
                        for i in range(3)
                    ]
        self.last_stamp = stamp
        self.last_position = body_position

        body = Odometry()
        body.header = msg.header
        body.header.frame_id = self.frame_id
        body.child_frame_id = self.base_frame_id
        body.pose.pose.position.x, body.pose.pose.position.y, body.pose.pose.position.z = body_position
        body.pose.pose.orientation.x, body.pose.pose.orientation.y = q_body[0], q_body[1]
        body.pose.pose.orientation.z, body.pose.pose.orientation.w = q_body[2], q_body[3]
        body.pose.covariance = msg.pose.covariance
        body.twist.twist.linear.x, body.twist.twist.linear.y, body.twist.twist.linear.z = self.filtered_velocity
        body.twist.covariance = msg.twist.covariance
        self.body_pub.publish(body)
        self.count += 1
        if self.count == 1 or self.count % 50 == 0:
            self.get_logger().info(
                'body=(%.2f, %.2f, %.2f), input healthy' % body_position)


def main(args=None):
    rclpy.init(args=args)
    node = LioPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
