#!/usr/bin/env python3
"""Relay SCAN odometry to Nav2 with a compatible reliable QoS.

The SCAN/LIO bridge intentionally publishes sensor data as best effort. Nav2's
controller and velocity smoother request reliable odometry, so DDS correctly
refuses to connect them directly. This small relay keeps the SCAN data path
unchanged while giving Nav2 a reliable local odometry topic.
"""

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class OdomQoSBridge(Node):

    def __init__(self):
        super().__init__('go2_nav2_odom_qos_bridge')
        self.declare_parameter('input_topic', '/scan_planner/body_pose')
        self.declare_parameter('output_topic', '/go2/nav2/odom')
        self.declare_parameter('stale_timeout', 0.5)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.stale_timeout = max(
            0.1, float(self.get_parameter('stale_timeout').value))
        if input_topic == output_topic:
            raise ValueError('odom QoS bridge input and output must differ')

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.output = self.create_publisher(Odometry, output_topic, output_qos)
        self.status_output = self.create_publisher(
            String, '/go2/nav2/odom_status', 10)
        self.create_subscription(
            Odometry, input_topic, self.on_odometry, input_qos)
        self.create_timer(1.0, self.publish_status)

        self.received = 0
        self.last_received = None
        self.get_logger().info(
            f'Nav2 odom QoS bridge: {input_topic} (best effort) -> '
            f'{output_topic} (reliable)')

    def on_odometry(self, message):
        self.last_received = time.monotonic()
        self.received += 1
        self.output.publish(message)

    def publish_status(self):
        now = time.monotonic()
        age = None if self.last_received is None else now - self.last_received
        if age is None:
            state = 'waiting'
        elif age > self.stale_timeout:
            state = 'stale'
        else:
            state = 'online'
        message = String()
        message.data = json.dumps({
            'state': state,
            'age': None if age is None else round(age, 3),
            'samples': self.received,
        }, separators=(',', ':'))
        self.status_output.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = OdomQoSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
