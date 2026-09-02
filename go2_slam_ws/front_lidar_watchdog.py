#!/usr/bin/env python3
"""Fail-safe watchdog for the ROS 1 front-lidar Point-LIO process.

Point-LIO assumes a continuous, monotonic lidar/IMU stream.  A ROS bridge
restart can reconnect both inputs without resetting the estimator, which may
leave it integrating an invalid state.  Restart Point-LIO only after both
inputs have recovered; also reject non-finite or physically impossible odom
jumps.  Restarting intentionally clears the unfinished in-memory map.
"""

import math
import subprocess
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


INPUT_STALE_SEC = 2.0
INPUT_RECOVERED_SEC = 0.6
STARTUP_GRACE_SEC = 10.0
RESTART_COOLDOWN_SEC = 15.0
MAX_ODOM_SPEED_MPS = 8.0


class FrontLidarWatchdog:
    def __init__(self):
        now = time.monotonic()
        self.started = now
        self.last_cloud = None
        self.last_imu = None
        self.last_odom = None
        self.last_odom_wall = None
        self.input_interrupted = False
        self.last_restart = -1e9
        rospy.Subscriber('/utlidar/cloud', PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber('/utlidar/imu', Imu, self.on_imu, queue_size=10)
        rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.on_odom, queue_size=5)
        rospy.Timer(rospy.Duration(0.5), self.check_inputs)

    def on_cloud(self, _msg):
        self.last_cloud = time.monotonic()

    def on_imu(self, _msg):
        self.last_imu = time.monotonic()

    def on_odom(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        current = (float(p.x), float(p.y), float(p.z))
        invalid = not all(math.isfinite(value) for value in current)
        if self.last_odom is not None and self.last_odom_wall is not None:
            dt = max(1e-3, now - self.last_odom_wall)
            speed = math.dist(self.last_odom, current) / dt
            invalid = invalid or speed > MAX_ODOM_SPEED_MPS
        self.last_odom = current
        self.last_odom_wall = now
        if invalid:
            self.restart_pointlio('检测到不可能的里程计跳变')

    def check_inputs(self, _event):
        now = time.monotonic()
        if now - self.started < STARTUP_GRACE_SEC:
            return
        cloud_age = float('inf') if self.last_cloud is None else now - self.last_cloud
        imu_age = float('inf') if self.last_imu is None else now - self.last_imu
        if cloud_age > INPUT_STALE_SEC or imu_age > INPUT_STALE_SEC:
            self.input_interrupted = True
            return
        if (self.input_interrupted and cloud_age < INPUT_RECOVERED_SEC and
                imu_age < INPUT_RECOVERED_SEC):
            self.input_interrupted = False
            self.restart_pointlio('雷达/IMU 数据中断后已恢复')

    def restart_pointlio(self, reason):
        now = time.monotonic()
        if now - self.last_restart < RESTART_COOLDOWN_SEC:
            return
        self.last_restart = now
        self.last_odom = None
        self.last_odom_wall = None
        rospy.logerr('%s；清除未保存地图并重启 Point-LIO', reason)
        subprocess.run(
            ['systemctl', '--user', 'restart', 'go2-front-pointlio.service'],
            check=False, timeout=10.0)


def main():
    rospy.init_node('go2_front_lidar_watchdog')
    FrontLidarWatchdog()
    rospy.spin()


if __name__ == '__main__':
    main()
