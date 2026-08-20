#!/usr/bin/env python3
"""Send one goal in front of the robot on the isolated SCAN-Planner dry-run graph."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class ForwardTestGoal(Node):
    def __init__(self):
        super().__init__('scan_planner_forward_test_goal')
        self.declare_parameter('distance', 1.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('body_height', 0.45)
        self.distance = float(self.get_parameter('distance').value)
        self.yaw_offset = math.radians(
            float(self.get_parameter('yaw_offset_deg').value))
        self.body_height = float(self.get_parameter('body_height').value)
        if not 0.1 <= self.distance <= 3.0:
            raise ValueError('distance must be between 0.1 and 3.0 metres')
        if abs(self.yaw_offset) > math.pi:
            raise ValueError('yaw_offset_deg must be between -180 and 180')
        self.pose = None
        self.sub = self.create_subscription(
            Odometry, '/scan_planner/body_pose', self.on_pose, qos_profile_sensor_data)
        self.pub = self.create_publisher(Path, '/scan_planner/global_path', 10)
        self.cancel_pub = self.create_publisher(Bool, '/scan_planner/cancel', 10)
        self.chassis_pub = self.create_publisher(
            Bool, '/scan_planner/chassis_enable', 10)

    def on_pose(self, msg):
        self.pose = msg

    def send(self):
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise RuntimeError('no /scan_planner/body_pose received')

        # Allow all reliable publisher/subscriber discovery handshakes to finish.
        discovery_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            if (self.pub.get_subscription_count() > 0 and
                    self.cancel_pub.get_subscription_count() > 0 and
                    self.chassis_pub.get_subscription_count() > 0):
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pub.get_subscription_count() < 1:
            raise RuntimeError('no /scan_planner/global_path subscriber')

        p = self.pose.pose.pose.position
        q = self.pose.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        heading = yaw + self.yaw_offset
        # Reference-path mode adds body_height before planning. Publish two
        # sparse waypoints at the body level expected by that callback.
        for fraction in (0.5, 1.0):
            waypoint = PoseStamped()
            waypoint.header = path.header
            waypoint.pose.position.x = (
                p.x + math.cos(heading) * self.distance * fraction)
            waypoint.pose.position.y = (
                p.y + math.sin(heading) * self.distance * fraction)
            waypoint.pose.position.z = p.z - self.body_height
            waypoint.pose.orientation.w = 1.0
            path.poses.append(waypoint)

        # Physically disarm first, even if the Web UI happened to be armed when
        # this diagnostic was launched. Then clear only the planner latch.
        self.chassis_pub.publish(Bool(data=False))
        safety_deadline = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < safety_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cancel_pub.publish(Bool(data=False))
        self.pub.publish(path)
        self.get_logger().info(
            'Dry-run goal %.2fm at %+.1fdeg -> (%.3f, %.3f, %.3f)' % (
                self.distance, math.degrees(self.yaw_offset),
                path.poses[-1].pose.position.x,
                path.poses[-1].pose.position.y,
                path.poses[-1].pose.position.z + self.body_height))
        # Spin briefly so the one-shot publication leaves the process reliably.
        end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardTestGoal()
    try:
        node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()
