#!/usr/bin/env python3
"""Publish low-overhead SCAN/Nav2 shadow comparison metrics."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String


def path_length(message):
    return sum(math.hypot(
        right.pose.position.x - left.pose.position.x,
        right.pose.position.y - left.pose.position.y)
        for left, right in zip(message.poses, message.poses[1:]))


class ShadowMonitor(Node):
    def __init__(self):
        super().__init__('go2_nav2_shadow_monitor')
        self.samples = {
            'scan_path': None,
            'nav2_path': None,
            'scan_cmd': None,
            'nav2_cmd': None,
        }
        self.output = self.create_publisher(
            String, '/go2/nav2/shadow_metrics', 10)
        self.backend = 'scan'
        self.chassis_enabled = False
        self.create_subscription(
            Path, '/scan_planner/global_path',
            lambda msg: self.on_path('scan_path', msg), 10)
        self.create_subscription(
            Path, '/plan', lambda msg: self.on_path('nav2_path', msg), 10)
        self.create_subscription(
            Twist, '/scan_planner/cmd_vel_test',
            lambda msg: self.on_cmd('scan_cmd', msg), 20)
        self.create_subscription(
            Twist, '/go2/nav2/cmd_vel_safe',
            lambda msg: self.on_cmd('nav2_cmd', msg), 20)
        self.create_subscription(
            String, '/scan_planner/chassis_status', self.on_chassis_status, 10)
        self.create_timer(0.5, self.publish)

    def on_chassis_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        backend = str(status.get('navigation_backend', 'scan')).lower()
        if backend in ('scan', 'nav2'):
            self.backend = backend
        self.chassis_enabled = bool(status.get('enabled'))

    def on_path(self, name, message):
        self.samples[name] = {
            'time': time.monotonic(),
            'frame': message.header.frame_id,
            'poses': len(message.poses),
            'length': path_length(message),
        }

    def on_cmd(self, name, message):
        self.samples[name] = {
            'time': time.monotonic(),
            'vx': float(message.linear.x),
            'vy': float(message.linear.y),
            'wz': float(message.angular.z),
        }

    def publish(self):
        now = time.monotonic()
        nav2_actuation = self.backend == 'nav2' and self.chassis_enabled
        result = {
            'mode': 'active' if nav2_actuation else 'shadow',
            'actuation': nav2_actuation,
            'selected_backend': self.backend,
        }
        for name, sample in self.samples.items():
            if sample is None:
                result[name] = None
                continue
            output = dict(sample)
            output.pop('time')
            output['age'] = round(now - sample['time'], 3)
            if 'length' in output:
                output['length'] = round(output['length'], 3)
            result[name] = output
        scan = self.samples['scan_cmd']
        nav2 = self.samples['nav2_cmd']
        if scan is not None and nav2 is not None:
            result['command_delta'] = {
                axis: round(nav2[axis] - scan[axis], 3)
                for axis in ('vx', 'vy', 'wz')
            }
        message = String()
        message.data = json.dumps(result, separators=(',', ':'))
        self.output.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ShadowMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
