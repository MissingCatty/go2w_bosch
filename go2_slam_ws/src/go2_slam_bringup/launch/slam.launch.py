"""GO2-W LIO-SAM 全链路的 host ROS2 节点。

前置：xt16_driver 与 Humble LIO-SAM 容器由 ~/go2_slam_ws/start.sh 启动。
本 launch 启动：
  - imu_attitude_bridge : /lowstate 500Hz 内置 IMU -> /dog_imu_lio 250Hz
  - lidar_preview_bridge: 原始 XT16 点云 C++ 抽稀，供 Web 预览
  - web_server          : 浏览器控制台 (http://<机器人IP>:8890)

参数：
  lidar_yaw_offset : 雷达前向相对狗前向的偏航角（度，正=逆时针/左），
                     默认 -90°（GO2-W 头盔 XT16 侧装，实测方向与出厂标定符号相反）。
                     用法: ros2 launch go2_slam_bringup slam.launch.py lidar_yaw_offset:=90
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yaw_offset = LaunchConfiguration('lidar_yaw_offset')
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_yaw_offset', default_value='-90.0',
            description='雷达前向相对狗前向的偏航角(度)'),
        Node(package='go2_imu_bridge', executable='imu_attitude_bridge',
             name='imu_attitude_bridge', output='screen'),
        Node(package='go2_imu_bridge', executable='lidar_preview_bridge',
             name='lidar_preview_bridge', output='screen'),
        Node(package='go2_slam_web', executable='web_server',
             name='web_server', output='screen',
             parameters=[{'lidar_yaw_offset': yaw_offset}]),
        # 实机底盘安全门常驻但默认锁定；只有 Web 显式启用后才转发速度。
        Node(package='go2_slam_web', executable='chassis_safety_gate',
             name='go2_chassis_safety_gate', output='screen'),
    ])
