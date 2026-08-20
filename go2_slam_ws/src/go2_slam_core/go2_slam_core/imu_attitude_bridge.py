#!/usr/bin/env python3
"""把 GO2-W 原生 LowState IMU 转换成 LIO-SAM 使用的 ROS Imu。

/lowstate 以约 500 Hz 发布完整的机身 IMU（四元数、角速度、加速度）。直接
使用它可以避开旧 /dog_imu_raw 的零四元数，也不再依赖 Unitree 旧 SLAM 进程。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState


class ImuAttitudeBridge(Node):

    CALIBRATION_SAMPLES = 1000
    GRAVITY = 9.46036

    def __init__(self):
        super().__init__('imu_attitude_bridge')
        self._published = 0
        self._input_seq = 0
        self._calibration_count = 0
        self._acc_residual_sum = [0.0, 0.0, 0.0]
        self._gyro_sum = [0.0, 0.0, 0.0]
        self._acc_bias = [0.0, 0.0, 0.0]
        self._gyro_bias = [0.0, 0.0, 0.0]
        self.pub = None

        # Unitree 的裸 DDS 发布端是 reliable；显式匹配，避免某些 CycloneDDS
        # 版本在高频话题上无法与默认 sensor_data QoS 建立连接。
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LowState, '/lowstate', self._on_lowstate, input_qos)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            'IMU 姿态桥启动: 正在用 2 秒静止样本标定内置 IMU')

    def _on_lowstate(self, msg):
        # 机身源为 500 Hz；LIO 配合 10 Hz 机械雷达使用 250 Hz 已有充足余量，
        # 隔帧可显著降低 Python 序列化和容器 DDS 回调的 CPU 占用。标定阶段仍
        # 使用全部样本。
        if self.pub is not None:
            self._input_seq += 1
            if self._input_seq % 2:
                return
        imu = msg.imu_state
        q = imu.quaternion
        norm = math.sqrt(sum(float(v) * float(v) for v in q))
        if not math.isfinite(norm) or norm < 0.5:
            return

        qx = float(q[1]) / norm
        qy = float(q[2]) / norm
        qz = float(q[3]) / norm
        qw = float(q[0]) / norm

        if self.pub is None:
            gyro = [float(v) for v in imu.gyroscope]
            acc = [float(v) for v in imu.accelerometer]
            # 只接受近似静止的样本，避免误把启动时的机器人运动标成零偏。
            if (math.sqrt(sum(v * v for v in gyro)) > 0.1 or
                    not 8.0 < math.sqrt(sum(v * v for v in acc)) < 11.0):
                return
            # q 表示 body->world；R 的第三行即 world-z 在 body 中的方向。
            expected = [
                self.GRAVITY * 2.0 * (qx * qz - qw * qy),
                self.GRAVITY * 2.0 * (qy * qz + qw * qx),
                self.GRAVITY * (1.0 - 2.0 * (qx * qx + qy * qy)),
            ]
            for i in range(3):
                self._acc_residual_sum[i] += acc[i] - expected[i]
                self._gyro_sum[i] += gyro[i]
            self._calibration_count += 1
            if self._calibration_count < self.CALIBRATION_SAMPLES:
                return
            self._acc_bias = [
                v / self._calibration_count for v in self._acc_residual_sum]
            self._gyro_bias = [
                v / self._calibration_count for v in self._gyro_sum]
            self.pub = self.create_publisher(
                Imu, '/dog_imu_lio', qos_profile_sensor_data)
            self.get_logger().info(
                '内置 IMU 标定完成: acc_bias=[%.4f, %.4f, %.4f], '
                'gyro_bias=[%.5f, %.5f, %.5f]' %
                (*self._acc_bias, *self._gyro_bias))
            return

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        # Unitree IMUState 四元数顺序为 [w, x, y, z]；ROS 为 [x,y,z,w]。
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.angular_velocity.x = float(imu.gyroscope[0]) - self._gyro_bias[0]
        out.angular_velocity.y = float(imu.gyroscope[1]) - self._gyro_bias[1]
        out.angular_velocity.z = float(imu.gyroscope[2]) - self._gyro_bias[2]
        out.linear_acceleration.x = float(imu.accelerometer[0]) - self._acc_bias[0]
        out.linear_acceleration.y = float(imu.accelerometer[1]) - self._acc_bias[1]
        out.linear_acceleration.z = float(imu.accelerometer[2]) - self._acc_bias[2]

        # LowState 未携带协方差；使用与设备噪声量级相符的保守对角值。
        out.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.02]
        out.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                           0.0, 0.001, 0.0,
                                           0.0, 0.0, 0.001]
        out.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
        self.pub.publish(out)
        self._published += 1

    def _report(self):
        if self.pub is None:
            self.get_logger().info(
                f'IMU 标定中: {self._calibration_count}/{self.CALIBRATION_SAMPLES}')
        else:
            self.get_logger().info(
                f'IMU 姿态桥: {self._published / 5.0:.1f} Hz')
        self._published = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImuAttitudeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
