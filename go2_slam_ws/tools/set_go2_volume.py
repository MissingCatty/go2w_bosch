#!/usr/bin/env python3
"""Set and verify the Go2 VUI speaker volume through the native ROS2 API."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response


SET_VOLUME = 1003
GET_VOLUME = 1004


class VuiVolumeClient(Node):
    def __init__(self):
        super().__init__('go2_vui_volume_client')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.responses = {}
        self.publisher = self.create_publisher(
            Request, '/api/vui/request', qos)
        self.subscription = self.create_subscription(
            Response, '/api/vui/response', self._on_response, qos)

    def _on_response(self, message):
        identity = message.header.identity
        self.responses[(int(identity.id), int(identity.api_id))] = message

    def wait_connected(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while self.publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
        if self.publisher.get_subscription_count() < 1:
            raise RuntimeError('Go2 VUI 服务未连接')

    def call(self, api_id, values, timeout=4.0):
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = int(api_id)
        request.header.policy.noreply = False
        request.parameter = json.dumps(values, separators=(',', ':'))
        key = (int(request.header.identity.id), int(api_id))
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                # The robot VUI request reader is best-effort, so resend the
                # same request identity at a bounded rate until acknowledged.
                self.publisher.publish(request)
                next_publish = now + 0.25
            rclpy.spin_once(self, timeout_sec=0.05)
            response = self.responses.pop(key, None)
            if response is not None:
                return int(response.header.status.code), response.data
        raise TimeoutError('Go2 VUI API %d 响应超时' % api_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--percent', type=int, default=10,
        help='speaker volume in 10%% increments, from 0 to 100 (default: 10)')
    args = parser.parse_args()
    if args.percent < 0 or args.percent > 100 or args.percent % 10:
        parser.error('--percent 必须是 0 到 100 之间的 10 的倍数')
    target = args.percent // 10

    rclpy.init()
    node = VuiVolumeClient()
    try:
        node.wait_connected()
        code, data = node.call(SET_VOLUME, {'volume': target})
        if code != 0:
            raise RuntimeError('设置音量失败: code=%d data=%s' % (code, data))
        time.sleep(0.20)
        code, data = node.call(GET_VOLUME, {})
        if code != 0:
            raise RuntimeError('读取音量失败: code=%d data=%s' % (code, data))
        actual = int(json.loads(data)['volume'])
        if actual != target:
            raise RuntimeError(
                '音量回读不一致: 期望 %d，实际 %d' % (target, actual))
        print('Go2 音量已设置并回读确认: %d%%（等级 %d/10）' % (
            actual * 10, actual))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

