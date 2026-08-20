"""宇树内置 SLAM 的 RPC 客户端（基于 unitree_api 消息，通道 /api/slam_operate/*）。

协议来源：宇树出厂 module 自带的 example/src/keyDemo.cpp：
  - START_MAPPING   = 1801  {"data": {"slam_type": "indoor"}}
  - END_MAPPING     = 1802  {"data": {"address": "/path/map.pcd"}}  (结束建图并保存 PCD)
  - STOP_NODE       = 1901  {"data": {}}
  - START_RELOCATION= 1804  (重定位)
"""

import threading

from unitree_api.msg import Request, Response

API_START_MAPPING = 1801
API_END_MAPPING = 1802
API_START_RELOCATION = 1804
API_STOP_NODE = 1901

REQ_TOPIC = '/api/slam_operate/request'
RESP_TOPIC = '/api/slam_operate/response'


class SlamRpcClient:
    """对 unitree_slam 服务做请求-应答调用（按 request id 匹配响应）。"""

    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(Request, REQ_TOPIC, 10)
        self._sub = node.create_subscription(Response, RESP_TOPIC, self._on_resp, 10)
        self._lock = threading.Lock()
        self._pending = {}   # rid -> (event, result_dict)
        self._seq = 0

    def _on_resp(self, msg):
        rid = msg.header.identity.id
        self._node.get_logger().info('RPC响应到达 id=%d code=%d' % (rid, msg.header.status.code))
        with self._lock:
            entry = self._pending.pop(rid, None)
        if entry is not None:
            ev, res = entry
            res['code'] = msg.header.status.code
            res['data'] = msg.data
            ev.set()
        else:
            self._node.get_logger().warn('RPC响应 id=%d 无匹配请求' % rid)

    def call(self, api_id, parameter='{"data": {}}', timeout=25.0):
        """同步调用。返回 (status_code, 响应 data JSON 字符串)；超时返回 (-1, 'timeout')。
        注意：宇树 slam 服务器响应有 ~8s 的固有延迟，超时需给足。"""
        with self._lock:
            self._seq += 1
            rid = self._seq
            ev, res = threading.Event(), {}
            self._pending[rid] = (ev, res)
        req = Request()
        req.header.identity.id = rid
        req.header.identity.api_id = api_id
        req.header.lease.id = 1
        req.header.policy.noreply = False
        req.parameter = parameter
        self._pub.publish(req)
        if not ev.wait(timeout):
            with self._lock:
                self._pending.pop(rid, None)
            return -1, 'timeout'
        return res['code'], res['data']

    def server_alive(self):
        """探测 slam 服务是否在线（调用 STOP_NODE 空操作，它总是立即响应）。"""
        code, _ = self.call(API_STOP_NODE, timeout=3.0)
        return code == 0
