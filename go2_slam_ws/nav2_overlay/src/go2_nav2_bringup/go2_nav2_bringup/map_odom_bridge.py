#!/usr/bin/env python3
"""Broadcast the Web-validated navigation-map alignment as a live TF.

The Web process owns alignment validity.  This node deliberately publishes a
dynamic transform instead of a static transform: if alignment is invalidated,
the transform becomes stale and Nav2 stops rather than continuing on an old
boot/session alignment.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


class MapOdomBridge(Node):
    def __init__(self):
        super().__init__('go2_nav2_map_odom_bridge')
        self.declare_parameter('input_topic', '/go2/nav2/map_to_odom')
        self.declare_parameter('alignment_topic', '/scan_planner/alignment_valid')
        self.declare_parameter('max_input_age', 0.75)
        self.declare_parameter('publish_frequency', 20.0)

        self.max_input_age = float(self.get_parameter('max_input_age').value)
        frequency = max(1.0, float(
            self.get_parameter('publish_frequency').value))
        self.broadcaster = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(
            String, '/go2/nav2/map_odom_status', 10)
        self.create_subscription(
            TransformStamped,
            str(self.get_parameter('input_topic').value),
            self.on_transform, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter('alignment_topic').value),
            self.on_alignment, 10)
        self.transform = None
        self.transform_t = None
        self.alignment_valid = False
        self.last_state = None
        self.create_timer(1.0 / frequency, self.tick)
        self.create_timer(0.5, self.publish_status)

    def on_transform(self, message):
        values = (
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
            message.transform.rotation.x,
            message.transform.rotation.y,
            message.transform.rotation.z,
            message.transform.rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error('Rejected non-finite navigation map transform')
            return
        if (message.header.frame_id != 'nav_map' or
                message.child_frame_id != 'odom'):
            self.get_logger().error(
                'Rejected transform %s -> %s; expected nav_map -> odom' % (
                    message.header.frame_id, message.child_frame_id))
            return
        self.transform = message
        self.transform_t = time.monotonic()

    def on_alignment(self, message):
        self.alignment_valid = bool(message.data)

    def state(self):
        now = time.monotonic()
        age = None if self.transform_t is None else now - self.transform_t
        if not self.alignment_valid:
            return 'alignment_invalid', age
        if self.transform is None:
            return 'transform_missing', age
        if age > self.max_input_age:
            return 'transform_stale', age
        return 'broadcasting', age

    def tick(self):
        state, _ = self.state()
        if state != 'broadcasting':
            return
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'nav_map'
        message.child_frame_id = 'odom'
        message.transform = self.transform.transform
        self.broadcaster.sendTransform(message)

    def publish_status(self):
        state, age = self.state()
        if state != self.last_state:
            # rclpy keys a call site by source location and rejects changing
            # its severity later.  Keep INFO and WARN on distinct lines.
            if state == 'broadcasting':
                self.get_logger().info(
                    'Nav2 map alignment state: %s' % state)
            else:
                self.get_logger().warn(
                    'Nav2 map alignment state: %s' % state)
            self.last_state = state
        message = String()
        message.data = json.dumps({
            'state': state,
            'alignment_valid': self.alignment_valid,
            'input_age': None if age is None else round(age, 3),
            'parent': 'nav_map',
            'child': 'odom',
        }, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
