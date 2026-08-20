#!/usr/bin/env python3
# 合成点云：模拟 8m×6m 房间 + 缓慢旋转/平移（模拟机器人行走），供全链路验证
import rclpy, math, struct, time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

def make_scene():
    """非对称走廊+家具场景，避免扫描匹配退化"""
    pts = []
    # 走廊墙体（非对称：有开口）
    for x in range(-80, 81):
        pts += [(x*0.1, -3.0, 0.0)]
    for x in range(-80, -10):
        pts += [(x*0.1, 3.0, 0.0)]
    for x in range(10, 81):
        pts += [(x*0.1, 3.0, 0.0)]
    for y in range(-30, 31):
        pts += [(-8.0, y*0.1, 0.0), (8.0, y*0.1, 0.0)]
    # 家具：不同位置的盒子/桌子
    boxes = [(1.0, -1.5, 0.8), (-1.5, 1.2, 0.6), (3.5, 0.5, 1.0), (-3.0, -0.5, 0.5),
             (0.5, 2.2, 0.4), (-5.5, -1.8, 0.9), (5.0, -2.2, 0.7), (-6.5, 0.8, 0.6)]
    for (bx, by, bs) in boxes:
        for i in range(0, 40):
            a = math.radians(i * 9)
            pts.append((bx + bs*math.cos(a), by + bs*math.sin(a), 0.0))
    # 一条斜桌角
    for i in range(0, 30):
        pts.append((6.0 - 0.04*i, -1.0 + 0.07*i, 0.0))
    return pts


class Synth(Node):
    def __init__(self):
        super().__init__('synth_lidar')
        self.pub = self.create_publisher(PointCloud2, '/unitree/slam_lidar/points', 10)
        self.scene = make_scene()
        self.t = 0.0
        self.create_timer(0.1, self.cb)  # 10Hz
    def cb(self):
        self.t += 0.1
        yaw = 0.03 * self.t       # 缓慢旋转
        dx = 0.05 * self.t        # 前进
        dy = 0.06 * math.sin(0.1 * self.t)  # 轻微S形
        cy, sy = math.cos(yaw), math.sin(yaw)
        # 模拟 GO2-W 侧装雷达：发布"雷达系"点云（场景相对雷达前向旋转 -90°）
        # 与 lidar_yaw_offset=90 的校准链路配合，整条链路验证一致性
        c90, s90 = math.cos(-math.pi/2), math.sin(-math.pi/2)
        buf = bytearray()
        for (x, y, z) in self.scene:
            # 1) 世界系运动变换（狗在行走）
            wx = x*cy - y*sy + dx
            wy = x*sy + y*cy + dy
            # 2) 转到雷达系（狗系 -> 雷达系，yaw=-90°）
            lx = wx*c90 - wy*s90
            ly = wx*s90 + wy*c90
            buf += struct.pack('<ffff', lx, ly, z, 100.0)
        m = PointCloud2()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'rslidar'
        m.height, m.width = 1, len(self.scene)
        m.fields = []
        for name, off in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12)):
            f = PointField()
            f.name, f.offset, f.datatype, f.count = name, off, PointField.FLOAT32, 1
            m.fields.append(f)
        m.is_bigendian = False
        m.point_step, m.row_step = 16, 16 * len(self.scene)
        m.is_dense = True
        m.data = bytes(buf)
        self.pub.publish(m)

rclpy.init(); n = Synth()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
n.destroy_node(); rclpy.shutdown()
