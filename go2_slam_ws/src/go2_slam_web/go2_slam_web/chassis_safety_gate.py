#!/usr/bin/env python3
"""Fail-closed bridge from one selected navigation backend to GO2-W Sport API.

The node starts locked.  A fresh explicit arm request is required after every
cancel, health fault or process restart.  While armed it also requires fresh
SCAN commands, LIO body odometry and the robot's built-in LowState IMU.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from unitree_api.msg import Request, Response
from unitree_go.msg import LowState, SportModeState


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003
STAND_DOWN_API_ID = 1005
STAND_UP_API_ID = 1004
BALANCE_STAND_API_ID = 1002
STAND_UP_MIN_SECONDS = 3.0
RECOVERY_MIN_SECONDS = 2.0
LIE_DOWN_MIN_SECONDS = 2.0
POSTURE_STAGE_TIMEOUT = 8.0
SPORT_REQUEST_RETRY_SECONDS = 0.25
SPORT_REQUEST_RETRY_TIMEOUT = 5.0
JOINT_LOCK_MODE = 6
LIE_DOWN_MODE = 5
MOVABLE_SPORT_MODES = (0, 1, 3)
POSTURE_ACTIONS = {
    'stand_down': (STAND_DOWN_API_ID, '卧倒'),
    'recovery_stand': (STAND_UP_API_ID, '两阶段恢复可移动姿态'),
}


def clamp(value, limit):
    return max(-limit, min(limit, float(value)))


def planner_command_ready(command_t, command_epoch_t):
    """Accept only commands produced after the current control epoch."""
    return (command_t is not None and command_epoch_t is not None and
            command_t >= command_epoch_t)


def planner_command_start_timed_out(command_t, command_epoch_t, now, timeout):
    """Report a missing first command after a bounded zero-speed grace."""
    if planner_command_ready(command_t, command_epoch_t):
        return False
    if command_epoch_t is None:
        return True
    return now - command_epoch_t > max(0.0, float(timeout))


class ChassisSafetyGate(Node):
    """Gate planner commands behind arming, watchdogs and velocity limits."""

    def __init__(self):
        super().__init__('go2_chassis_safety_gate')
        # Autonomous navigation and manual keyboard control have independent
        # limits. Navigation is allowed to use the faster wheel-driving
        # profile without silently increasing direct teleoperation speed.
        self.max_vx = float(self.declare_parameter('max_vx', 0.80).value)
        self.teleop_max_vx = float(
            self.declare_parameter('teleop_max_vx', 0.40).value)
        self.max_vy = float(self.declare_parameter('max_vy', 0.10).value)
        self.max_vyaw = float(self.declare_parameter('max_vyaw', 0.45).value)
        self.max_accel = float(self.declare_parameter('max_accel', 0.60).value)
        # Acceleration stays gentle, while a lower planner target is allowed
        # to brake substantially faster. Emergency live-obstacle stops bypass
        # both slew limits below.
        self.max_decel = float(self.declare_parameter('max_decel', 1.50).value)
        self.max_yaw_accel = float(self.declare_parameter('max_yaw_accel', 0.60).value)
        self.max_yaw_decel = float(
            self.declare_parameter('max_yaw_decel', 1.20).value)
        self.emergency_stop_timeout = float(
            self.declare_parameter('emergency_stop_timeout', 0.20).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.30).value)
        # Nav2 intentionally emits no velocity while it has no active goal.
        # Start this grace period when Web releases cancellation for a fresh
        # goal, not when the operator arms the otherwise idle chassis.
        self.initial_cmd_timeout = max(
            self.cmd_timeout,
            float(self.declare_parameter('initial_cmd_timeout', 3.0).value))
        self.teleop_timeout = float(self.declare_parameter('teleop_timeout', 0.35).value)
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 0.75).value)
        self.lowstate_timeout = float(self.declare_parameter('lowstate_timeout', 1.00).value)
        self.heartbeat_timeout = float(self.declare_parameter('heartbeat_timeout', 1.00).value)
        self.max_tilt = float(self.declare_parameter('max_tilt', 0.55).value)

        default_backend = str(
            self.declare_parameter('navigation_backend', 'nav2').value).lower()
        self.navigation_backend = (
            default_backend if default_backend in ('scan', 'nav2') else 'nav2')
        self.backend_switch_error = ''

        self.request_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(
            Response, '/api/sport/response', self.on_sport_response, 10)
        self.enabled_pub = self.create_publisher(Bool, '/scan_planner/chassis_enabled', 10)
        self.status_pub = self.create_publisher(String, '/scan_planner/chassis_status', 10)
        self.create_subscription(
            Twist, '/scan_planner/cmd_vel_test', self.on_scan_command, 20)
        self.create_subscription(
            Twist, '/go2/nav2/cmd_vel_safe', self.on_nav2_command, 20)
        self.create_subscription(
            String, '/go2/navigation/backend', self.on_navigation_backend, 10)
        self.create_subscription(
            Bool, '/scan_planner/emergency_stop', self.on_emergency_stop, 20)
        self.create_subscription(
            Twist, '/scan_planner/teleop_cmd', self.on_teleop_command, 20)
        self.create_subscription(
            Twist, '/scan_planner/recovery_cmd', self.on_recovery_command, 20)
        self.create_subscription(
            Bool, '/scan_planner/recovery_active', self.on_recovery_active, 10)
        self.create_subscription(
            Bool, '/scan_planner/planning/local_waiting',
            self.on_local_waiting, 10)
        self.create_subscription(
            Bool, '/scan_planner/teleop_enable', self.on_teleop_enable, 10)
        self.create_subscription(
            Odometry, '/scan_planner/body_pose', self.on_odometry,
            qos_profile_sensor_data)
        # The transformed body pose is republished from the Humble SCAN
        # container.  Fast-DDS best-effort delivery across the container/host
        # boundary can occasionally drop it for longer than the navigation
        # watchdog even while the original LIO stream remains healthy.  The
        # safety gate only needs an independent localization heartbeat here,
        # so retain body_pose and also listen to its raw LIO source.
        self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.on_raw_odometry,
            qos_profile_sensor_data)
        self.create_subscription(
            # The gate checks tilt, not high-rate state estimation.  Use the
            # robot's 20 Hz low-frequency mirror; LIO's IMU bridge remains on
            # the original ~200 Hz /lowstate stream.
            LowState, '/lf/lowstate', self.on_lowstate,
            qos_profile_sensor_data)
        self.create_subscription(
            SportModeState, '/lf/sportmodestate', self.on_sport_state,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/scan_planner/chassis_enable', self.on_enable, 10)
        self.create_subscription(
            Bool, '/scan_planner/chassis_heartbeat', self.on_heartbeat, 10)
        self.create_subscription(
            Bool, '/scan_planner/alignment_valid', self.on_alignment, 10)
        self.create_subscription(
            Bool, '/scan_planner/cancel', self.on_cancel, 10)
        self.create_subscription(
            String, '/scan_planner/posture_command',
            self.on_posture_command, 10)

        self.armed = False
        self.cancelled = True
        self.teleop_enabled = False
        self.planner_cmds = {
            'scan': (0.0, 0.0, 0.0),
            'nav2': (0.0, 0.0, 0.0),
        }
        self.teleop_cmd = (0.0, 0.0, 0.0)
        self.recovery_cmd = (0.0, 0.0, 0.0)
        self.output = (0.0, 0.0, 0.0)
        self.planner_cmd_times = {'scan': None, 'nav2': None}
        self.teleop_cmd_t = None
        self.recovery_cmd_t = None
        self.recovery_active = False
        self.local_waiting = False
        self.emergency_stop = False
        self.emergency_stop_t = None
        self.emergency_stop_applied = False
        self.odom_t = None
        self.raw_odom_t = None
        self.lowstate_t = None
        self.heartbeat_t = None
        self.alignment_t = None
        self.alignment_valid = False
        self.roll = 0.0
        self.pitch = 0.0
        self.armed_t = None
        self.navigation_started_t = None
        self.last_tick_t = time.monotonic()
        self.stop_burst = 0
        self.cancel_seq = 0
        self.posture_seq = 0
        self.pending_posture = None
        # Reconcile this from SportModeState after startup.  Assuming movable
        # would be unsafe when the robot survived a process restart in mode=5
        # (lieDown) or mode=6 (jointLock).
        self.posture_state = 'unknown'
        self.posture_stage_started_t = None
        self.posture_stage_deadline = None
        self.pending_sport_request = None
        self.last_sport_response_code = None
        self.sport_mode = None
        self.sport_progress = None
        self.sport_state_t = None
        self.reason = '底盘已锁定'
        self.ever_armed = False

        self.create_timer(0.05, self.tick)
        self.create_timer(0.20, self.publish_status)
        self.get_logger().info(
            'GO2-W chassis gate ready and LOCKED: backend=%s, '
            'navigation %.2f/%.2f m/s, teleop %.2f/%.2f m/s, yaw %.2f rad/s' %
            (self.navigation_backend, self.max_vx, self.max_vy, self.teleop_max_vx,
             self.max_vy, self.max_vyaw))

    def store_planner_command(self, backend, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            if backend == self.navigation_backend:
                self.trip('收到%s非有限速度指令' % backend.upper())
            return
        self.planner_cmds[backend] = (
            clamp(values[0], self.max_vx),
            clamp(values[1], self.max_vy),
            clamp(values[2], self.max_vyaw),
        )
        self.planner_cmd_times[backend] = time.monotonic()

    def on_scan_command(self, msg):
        self.store_planner_command('scan', msg)

    def on_nav2_command(self, msg):
        self.store_planner_command('nav2', msg)

    def on_navigation_backend(self, msg):
        requested = str(msg.data or '').strip().lower()
        if requested not in ('scan', 'nav2'):
            self.backend_switch_error = '未知导航后端: %s' % requested
            self.get_logger().error(self.backend_switch_error)
            return
        if requested == self.navigation_backend:
            self.backend_switch_error = ''
            return
        # Backend handover is a maintenance operation. It is never accepted
        # while the physical gate is armed, even if both command streams are
        # currently zero, because their timestamps are independent.
        if self.armed:
            self.backend_switch_error = (
                '底盘启用期间拒绝从%s切换到%s' %
                (self.navigation_backend.upper(), requested.upper()))
            self.get_logger().error(self.backend_switch_error)
            return
        self.navigation_backend = requested
        self.backend_switch_error = ''
        self.cancelled = True
        self.planner_cmds = {
            'scan': (0.0, 0.0, 0.0),
            'nav2': (0.0, 0.0, 0.0),
        }
        self.planner_cmd_times = {'scan': None, 'nav2': None}
        self.navigation_started_t = None
        self.recovery_active = False
        self.recovery_cmd = (0.0, 0.0, 0.0)
        self.recovery_cmd_t = None
        self.local_waiting = False
        self.reason = '已选择%s，底盘保持锁定' % requested.upper()
        self.get_logger().warn(
            'Navigation backend selected: %s; chassis remains LOCKED' %
            requested)

    def on_emergency_stop(self, msg):
        self.emergency_stop = bool(msg.data)
        self.emergency_stop_t = time.monotonic()
        if not self.emergency_stop:
            self.emergency_stop_applied = False

    def on_teleop_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            self.trip('收到非有限键盘速度指令')
            return
        if not self.teleop_enabled:
            return
        self.teleop_cmd = (
            clamp(values[0], self.teleop_max_vx),
            clamp(values[1], self.max_vy),
            clamp(values[2], self.max_vyaw),
        )
        self.teleop_cmd_t = time.monotonic()

    def on_recovery_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            self.trip('收到非有限脱困速度指令')
            return
        self.recovery_cmd = (
            clamp(values[0], self.max_vx),
            clamp(values[1], self.max_vy),
            clamp(values[2], self.max_vyaw),
        )
        self.recovery_cmd_t = time.monotonic()

    def on_recovery_active(self, msg):
        self.recovery_active = bool(msg.data)
        if not self.recovery_active:
            self.recovery_cmd = (0.0, 0.0, 0.0)
            self.recovery_cmd_t = None

    def on_local_waiting(self, msg):
        self.local_waiting = bool(msg.data)
        if not self.local_waiting:
            self.recovery_active = False
            self.recovery_cmd = (0.0, 0.0, 0.0)
            self.recovery_cmd_t = None

    def on_teleop_enable(self, msg):
        enabled = bool(msg.data)
        if enabled:
            # Selecting teleop never arms the robot by itself.  The Web API
            # first cancels SCAN, selects this input, then separately requests
            # the normal guarded chassis arm operation.
            if self.armed:
                self.disarm('切换键盘控制，底盘已重新锁定', send_stop=True)
            self.teleop_enabled = True
            self.cancelled = True
            self.teleop_cmd = (0.0, 0.0, 0.0)
            self.teleop_cmd_t = None
            self.recovery_active = False
            self.recovery_cmd = (0.0, 0.0, 0.0)
            self.recovery_cmd_t = None
            self.reason = '键盘控制已选择，等待安全启用'
            self.publish_enabled()
            return
        was_enabled = self.teleop_enabled
        self.teleop_enabled = False
        self.teleop_cmd = (0.0, 0.0, 0.0)
        self.teleop_cmd_t = None
        if was_enabled or self.armed:
            self.disarm('键盘控制已关闭，底盘已锁定', send_stop=True)

    def on_odometry(self, _msg):
        self.odom_t = time.monotonic()

    def on_raw_odometry(self, _msg):
        self.raw_odom_t = time.monotonic()

    def navigation_odom_age(self, now=None):
        """Age of the freshest independent navigation pose source."""
        now = time.monotonic() if now is None else now
        ages = [
            now - stamp for stamp in (self.odom_t, self.raw_odom_t)
            if stamp is not None
        ]
        return min(ages) if ages else None

    def on_lowstate(self, msg):
        rpy = msg.imu_state.rpy
        if len(rpy) >= 2 and all(math.isfinite(float(value)) for value in rpy[:2]):
            self.roll = float(rpy[0])
            self.pitch = float(rpy[1])
            self.lowstate_t = time.monotonic()

    def on_sport_state(self, msg):
        progress = float(msg.progress)
        if not math.isfinite(progress):
            return
        self.sport_mode = int(msg.mode)
        self.sport_progress = progress
        self.sport_state_t = time.monotonic()
        if self.posture_state == 'unknown' and abs(progress) <= 1e-3:
            if self.sport_mode == LIE_DOWN_MODE:
                self.posture_state = 'stand_down'
                self.reason = '检测到机器狗处于卧倒状态，底盘保持锁定'
            elif self.sport_mode == JOINT_LOCK_MODE:
                self.posture_state = 'recovery_failed'
                self.reason = '检测到站立锁定 mode=6，请执行恢复可移动姿态'
            elif self.sport_mode in MOVABLE_SPORT_MODES:
                self.posture_state = 'movable'
                self.reason = '已确认可移动 Sport mode，底盘保持锁定'
            else:
                self.posture_state = 'recovery_failed'
                self.reason = '检测到非可移动 Sport mode=%d，底盘保持锁定' % self.sport_mode

    def on_sport_response(self, msg):
        pending = self.pending_sport_request
        if pending is None:
            return
        identity = msg.header.identity
        if (int(identity.id) != pending['id'] or
                int(identity.api_id) != pending['api_id']):
            return
        code = int(msg.header.status.code)
        self.last_sport_response_code = code
        if code != 0:
            # GO2-W commonly answers posture calls with -1 while its Sport
            # state machine is briefly busy (including immediately after a
            # StopMove).  Unitree's own go2w example keeps issuing the call in
            # this case.  Retry with a fresh request ID: resending the rejected
            # ID can replay a cached rejection.  Other error codes remain hard
            # failures, and -1 is still bounded by retry_deadline.
            now = time.monotonic()
            if code == -1 and now < pending['retry_deadline']:
                request = self.build_request(pending['api_id'], '', noreply=False)
                pending['message'] = request
                pending['id'] = int(request.header.identity.id)
                pending['rejection_count'] += 1
                pending['next_publish'] = now + SPORT_REQUEST_RETRY_SECONDS
                self.reason = (
                    'Sport API 暂时忙碌，正在重试姿态动作（第%d次）' %
                    pending['rejection_count'])
                self.get_logger().warn(
                    'POSTURE transient rejection api=%d code=-1; '
                    'retry=%d new_id=%d' % (
                        pending['api_id'], pending['rejection_count'],
                        pending['id']))
                return
            self.pending_sport_request = None
            self.fail_posture(
                'Sport API %d 拒绝姿态动作，错误码 %d' %
                (int(identity.api_id), code))
            return
        self.pending_sport_request = None
        self.get_logger().warn(
            'POSTURE Sport API acknowledged api=%d id=%d' %
            (int(identity.api_id), int(identity.id)))

    def health_fault(self, now=None):
        now = time.monotonic() if now is None else now
        if self.request_pub.get_subscription_count() < 1:
            return 'Sport API 未连接'
        if self.lowstate_t is None or now - self.lowstate_t > self.lowstate_timeout:
            return '机身 LowState 超时'
        if self.heartbeat_t is None or now - self.heartbeat_t > self.heartbeat_timeout:
            return 'Web 控制心跳超时'
        if abs(self.roll) > self.max_tilt or abs(self.pitch) > self.max_tilt:
            return '机身倾角超限'
        # Manual keyboard control is a direct body-frame velocity source.  It
        # does not consume map coordinates or navigation odometry, so only the
        # physical/communications watchdogs above apply.  Autonomous
        # navigation retains both localization checks.
        if not self.teleop_enabled:
            odom_age = self.navigation_odom_age(now)
            if odom_age is None or odom_age > self.odom_timeout:
                return '导航里程计超时'
            if (not self.alignment_valid or self.alignment_t is None or
                    now - self.alignment_t > self.heartbeat_timeout):
                return '本次开机的地图位姿未标定'
        return ''

    def on_enable(self, msg):
        if not msg.data:
            self.disarm('底盘已由网页锁定', send_stop=True)
            return
        if self.pending_posture is not None or self.posture_state != 'movable':
            posture_reasons = {
                'stand_down': '机器狗已卧倒，请先恢复可移动姿态',
                'locking_for_lie': '正在切换到站立锁定状态，随后将卧倒',
                'lying_down': '卧倒动作正在执行，请稍候',
                'standing_locked': '已站立但关节仍锁定，正在等待恢复',
                'recovering': '正在恢复可移动状态，请稍候',
                'recovery_failed': '恢复状态未确认，请重试恢复动作',
            }
            detail = posture_reasons.get(
                self.posture_state, '姿态动作正在准备')
            self.disarm('无法启用：' + detail, send_stop=False)
            return
        fault = self.health_fault()
        if fault:
            self.disarm('无法启用：' + fault, send_stop=False)
            return
        self.armed = True
        self.ever_armed = True
        self.armed_t = time.monotonic()
        self.navigation_started_t = None
        self.output = (0.0, 0.0, 0.0)
        if self.teleop_enabled:
            self.reason = '键盘控制已启用，等待按键'
        else:
            self.reason = '已启用，等待新导航目标' if self.cancelled else '底盘已启用'
        self.get_logger().warn('CHASSIS ARMED; waiting for fresh control commands')
        self.publish_enabled()

    def on_heartbeat(self, msg):
        if msg.data:
            self.heartbeat_t = time.monotonic()

    def on_alignment(self, msg):
        self.alignment_valid = bool(msg.data)
        self.alignment_t = time.monotonic()
        if not self.alignment_valid and self.armed and not self.teleop_enabled:
            self.trip('地图位姿标定失效')

    def on_cancel(self, msg):
        self.cancelled = bool(msg.data)
        if self.cancelled:
            self.navigation_started_t = None
            self.teleop_enabled = False
            self.teleop_cmd = (0.0, 0.0, 0.0)
            self.teleop_cmd_t = None
            self.recovery_active = False
            self.recovery_cmd = (0.0, 0.0, 0.0)
            self.recovery_cmd_t = None
            self.local_waiting = False
            self.cancel_seq += 1
            self.disarm('导航取消，底盘已锁定', send_stop=True)
            return
        # Web publishes cancel=False immediately before each fresh goal.  A
        # planner command from before this edge must never move the robot, and
        # Nav2 needs a bounded interval to accept the action, build its local
        # rollout and publish the first safe velocity.
        self.navigation_started_t = time.monotonic()
        self.output = (0.0, 0.0, 0.0)
        if self.armed and not self.teleop_enabled:
            self.reason = '已收到新目标，等待%s首条指令' % (
                self.navigation_backend.upper())

    def on_posture_command(self, msg):
        """Queue a one-shot posture action behind an unconditional stop burst."""
        action = str(msg.data or '').strip()
        command = POSTURE_ACTIONS.get(action)
        if command is None:
            self.get_logger().error('Rejected unknown posture action: %r' % action)
            return
        if (self.pending_posture is not None or
                self.posture_state in ('preparing', 'locking_for_lie', 'lying_down',
                                       'standing_locked', 'recovering')):
            self.get_logger().warn(
                'Rejected posture action %s: another action is pending' % action)
            return
        if self.request_pub.get_subscription_count() < 1:
            self.reason = '姿态动作失败：Sport API 未连接'
            self.publish_enabled()
            return

        now = time.monotonic()
        api_id, label = command
        # A posture action is an exclusive physical operation.  Invalidate
        # both navigation and teleop before the action can reach Sport API.
        self.cancelled = True
        self.navigation_started_t = None
        self.teleop_enabled = False
        self.planner_cmds = {
            'scan': (0.0, 0.0, 0.0),
            'nav2': (0.0, 0.0, 0.0),
        }
        self.teleop_cmd = (0.0, 0.0, 0.0)
        self.planner_cmd_times = {'scan': None, 'nav2': None}
        self.teleop_cmd_t = None
        self.recovery_active = False
        self.recovery_cmd = (0.0, 0.0, 0.0)
        self.recovery_cmd_t = None
        self.cancel_seq += 1
        self.disarm('正在准备%s，底盘已锁定' % label,
                    send_stop=False)
        # Always stop even if this gate has never armed since startup: another
        # Unitree command source may have left the robot in Move mode.
        self.stop_burst = max(self.stop_burst, 4)
        self.posture_seq += 1
        self.pending_posture = {
            'action': action,
            'api_id': api_id,
            'label': label,
            'not_before': now + 0.20,
        }
        self.posture_state = 'preparing'
        self.posture_stage_started_t = None
        self.posture_stage_deadline = None
        self.pending_sport_request = None
        self.last_sport_response_code = None
        self.get_logger().warn(
            'POSTURE %s queued; chassis locked and stop burst started' % action)

    def fail_posture(self, detail):
        self.pending_posture = None
        self.pending_sport_request = None
        self.posture_state = 'recovery_failed'
        self.posture_stage_started_t = None
        self.posture_stage_deadline = None
        self.reason = '姿态动作失败：' + detail + '；底盘保持锁定'
        self.get_logger().error(self.reason)
        self.publish_enabled()

    def disarm(self, reason, send_stop):
        was_armed = self.armed
        self.armed = False
        self.armed_t = None
        self.navigation_started_t = None
        self.output = (0.0, 0.0, 0.0)
        self.reason = reason
        if send_stop and (was_armed or self.ever_armed):
            self.stop_burst = max(self.stop_burst, 4)
        if was_armed:
            self.get_logger().warn(reason)
        self.publish_enabled()

    def trip(self, reason):
        self.cancelled = True
        self.navigation_started_t = None
        self.teleop_enabled = False
        self.teleop_cmd = (0.0, 0.0, 0.0)
        self.teleop_cmd_t = None
        self.disarm('安全停车：' + reason, send_stop=True)

    def publish_enabled(self):
        message = Bool()
        message.data = self.armed
        self.enabled_pub.publish(message)

    @staticmethod
    def build_request(api_id, parameter='', noreply=True):
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = int(api_id)
        request.header.policy.noreply = bool(noreply)
        request.parameter = parameter
        return request

    def publish_request(self, api_id, parameter='', noreply=True):
        request = self.build_request(api_id, parameter, noreply)
        self.request_pub.publish(request)

    def begin_posture_request(self, api_id, stage, now):
        """Reliably submit one best-effort Sport action with a stable ID."""
        request = self.build_request(api_id, '', noreply=False)
        self.pending_sport_request = {
            'message': request,
            'id': int(request.header.identity.id),
            'api_id': int(api_id),
            'stage': stage,
            'next_publish': now + SPORT_REQUEST_RETRY_SECONDS,
            'retry_deadline': now + SPORT_REQUEST_RETRY_TIMEOUT,
            'rejection_count': 0,
        }
        self.request_pub.publish(request)
        self.get_logger().warn(
            'POSTURE Sport API request sent api=%d id=%d stage=%s' %
            (int(api_id), int(request.header.identity.id), stage))

    def publish_move(self, vx, vy, vyaw):
        self.publish_request(MOVE_API_ID, json.dumps(
            {'x': float(vx), 'y': float(vy), 'z': float(vyaw)},
            separators=(',', ':')))

    def publish_stop(self):
        self.publish_move(0.0, 0.0, 0.0)
        self.publish_request(STOP_MOVE_API_ID, '{}', noreply=False)

    @staticmethod
    def slew(current, target, delta):
        return current + clamp(target - current, delta)

    @staticmethod
    def slew_with_braking(current, target, accel, decel, dt):
        braking = (
            abs(target) < abs(current) or
            (abs(current) > 1e-6 and current * target <= 0.0))
        return ChassisSafetyGate.slew(
            current, target, (decel if braking else accel) * dt)

    def tick(self):
        now = time.monotonic()
        dt = max(0.0, min(0.10, now - self.last_tick_t))
        self.last_tick_t = now

        if self.stop_burst > 0:
            self.publish_stop()
            self.stop_burst -= 1

        if (self.pending_posture is not None and self.stop_burst == 0 and
                now >= self.pending_posture['not_before']):
            command = self.pending_posture
            self.pending_posture = None
            if command['action'] == 'stand_down':
                # GO2-W rejects StandDown (1005) from the normal movable
                # mode=1.  The supported sequence is StandUp (1004) to the
                # joint-locked mode=6, then StandDown to mode=5.  If a prior
                # action already left the robot in mode=6, skip stage 1.
                fresh_state = (self.sport_state_t is not None and
                               now - self.sport_state_t < 1.0)
                already_locked = (
                    fresh_state and self.sport_mode == JOINT_LOCK_MODE and
                    abs(self.sport_progress) <= 1e-3)
                if already_locked:
                    self.begin_posture_request(
                        STAND_DOWN_API_ID, 'stand_down', now)
                    self.posture_state = 'lying_down'
                    self.reason = '已处于站立锁定状态，正在执行卧倒'
                else:
                    self.begin_posture_request(
                        STAND_UP_API_ID, 'lock_before_lie', now)
                    self.posture_state = 'locking_for_lie'
                    self.reason = '第一阶段：正在切换到站立关节锁定状态'
                self.posture_stage_started_t = now
                self.posture_stage_deadline = now + POSTURE_STAGE_TIMEOUT
            else:
                # Stage 1: StandUp deliberately ends in mode=6 (jointLock).
                # BalanceStand must not be sent until that physical action is
                # complete. If a previous attempt already reached jointLock,
                # resume directly from stage 2 without repeating StandUp.
                fresh_state = (self.sport_state_t is not None and
                               now - self.sport_state_t < 1.0)
                already_locked = (
                    fresh_state and self.sport_mode == JOINT_LOCK_MODE and
                    abs(self.sport_progress) <= 1e-3)
                if already_locked:
                    self.begin_posture_request(
                        BALANCE_STAND_API_ID, 'balance_stand', now)
                    self.posture_state = 'recovering'
                    self.posture_stage_started_t = now
                    self.posture_stage_deadline = now + POSTURE_STAGE_TIMEOUT
                    self.reason = '已经处于站立锁定状态，正在切换到可移动平衡站立'
                else:
                    self.begin_posture_request(
                        command['api_id'], 'stand_up', now)
                    self.posture_state = 'standing_locked'
                    self.posture_stage_started_t = now
                    self.posture_stage_deadline = now + POSTURE_STAGE_TIMEOUT
                    self.reason = '第一阶段：正在从卧倒恢复到站立锁定状态'
            self.get_logger().warn(
                'POSTURE %s sequence started; state=%s; chassis remains locked' %
                (command['action'], self.posture_state))

        if self.posture_state == 'locking_for_lie':
            elapsed = now - self.posture_stage_started_t
            fresh_state = (self.sport_state_t is not None and
                           self.sport_state_t >= self.posture_stage_started_t)
            lock_complete = (
                elapsed >= STAND_UP_MIN_SECONDS and fresh_state and
                self.sport_mode == JOINT_LOCK_MODE and
                abs(self.sport_progress) <= 1e-3)
            if lock_complete:
                self.pending_sport_request = None
                self.begin_posture_request(
                    STAND_DOWN_API_ID, 'stand_down', now)
                self.posture_state = 'lying_down'
                self.posture_stage_started_t = now
                self.posture_stage_deadline = now + POSTURE_STAGE_TIMEOUT
                self.reason = '第二阶段：站立锁定已确认，正在执行卧倒'
                self.get_logger().warn(
                    'POSTURE lie stage 1 confirmed mode=%d progress=%.3f; '
                    'StandDown sent (api=%d)' %
                    (self.sport_mode, self.sport_progress,
                     STAND_DOWN_API_ID))
            elif now >= self.posture_stage_deadline:
                self.fail_posture(
                    '卧倒前未确认站立锁定 mode=6（当前 mode=%r, progress=%r）' %
                    (self.sport_mode, self.sport_progress))

        elif self.posture_state == 'lying_down':
            elapsed = now - self.posture_stage_started_t
            fresh_state = (self.sport_state_t is not None and
                           self.sport_state_t >= self.posture_stage_started_t)
            lie_complete = (
                elapsed >= LIE_DOWN_MIN_SECONDS and fresh_state and
                self.sport_mode == LIE_DOWN_MODE and
                abs(self.sport_progress) <= 1e-3)
            if lie_complete:
                self.pending_sport_request = None
                self.posture_state = 'stand_down'
                self.posture_stage_started_t = None
                self.posture_stage_deadline = None
                self.reason = '卧倒动作已确认，底盘保持锁定'
                self.get_logger().warn(
                    'POSTURE StandDown confirmed mode=%d progress=%.3f' %
                    (self.sport_mode, self.sport_progress))
            elif now >= self.posture_stage_deadline:
                self.fail_posture(
                    '未确认卧倒 mode=5（当前 mode=%r, progress=%r）' %
                    (self.sport_mode, self.sport_progress))

        elif self.posture_state == 'standing_locked':
            elapsed = now - self.posture_stage_started_t
            fresh_state = (self.sport_state_t is not None and
                           self.sport_state_t >= self.posture_stage_started_t)
            stand_complete = (
                elapsed >= STAND_UP_MIN_SECONDS and fresh_state and
                self.sport_mode == JOINT_LOCK_MODE and
                abs(self.sport_progress) <= 1e-3)
            if stand_complete:
                # Stage 2: BalanceStand leaves jointLock and returns the
                # normal movable high-level controller state (mode=1).
                self.pending_sport_request = None
                self.begin_posture_request(
                    BALANCE_STAND_API_ID, 'balance_stand', now)
                self.posture_state = 'recovering'
                self.posture_stage_started_t = now
                self.posture_stage_deadline = now + POSTURE_STAGE_TIMEOUT
                self.reason = '第二阶段：已站立，正在从关节锁定恢复可移动状态'
                self.get_logger().warn(
                    'POSTURE stage 1 confirmed mode=%d progress=%.3f; '
                    'BalanceStand sent (api=%d)' %
                    (self.sport_mode, self.sport_progress,
                     BALANCE_STAND_API_ID))
            elif now >= self.posture_stage_deadline:
                self.fail_posture(
                    '未确认站立锁定 mode=6（当前 mode=%r, progress=%r）' %
                    (self.sport_mode, self.sport_progress))

        elif self.posture_state == 'recovering':
            elapsed = now - self.posture_stage_started_t
            fresh_state = (self.sport_state_t is not None and
                           self.sport_state_t >= self.posture_stage_started_t)
            recovery_complete = (
                elapsed >= RECOVERY_MIN_SECONDS and fresh_state and
                self.sport_mode in MOVABLE_SPORT_MODES and
                abs(self.sport_progress) <= 1e-3)
            if recovery_complete:
                self.pending_sport_request = None
                self.posture_state = 'movable'
                self.posture_stage_started_t = None
                self.posture_stage_deadline = None
                self.reason = '两阶段恢复完成：已进入可移动状态，底盘等待控制启用'
                self.get_logger().warn(
                    'POSTURE recovery confirmed movable mode=%d progress=%.3f' %
                    (self.sport_mode, self.sport_progress))
            elif now >= self.posture_stage_deadline:
                self.fail_posture(
                    '未确认可移动 mode（当前 mode=%r, progress=%r）' %
                    (self.sport_mode, self.sport_progress))

        pending_request = self.pending_sport_request
        if pending_request is not None and now >= pending_request['next_publish']:
            if now >= pending_request['retry_deadline']:
                self.get_logger().error(
                    'POSTURE Sport API response timeout api=%d id=%d; '
                    'waiting for physical mode confirmation' %
                    (pending_request['api_id'], pending_request['id']))
                self.pending_sport_request = None
            else:
                # The robot request reader is best-effort. Reuse the same ID so
                # duplicate delivery is idempotent and one response matches it.
                self.request_pub.publish(pending_request['message'])
                pending_request['next_publish'] = (
                    now + SPORT_REQUEST_RETRY_SECONDS)

        if not self.armed:
            return
        fault = self.health_fault(now)
        if fault:
            self.trip(fault)
            return
        emergency_active = (
            not self.teleop_enabled and self.emergency_stop and
            self.emergency_stop_t is not None and
            now - self.emergency_stop_t <= self.emergency_stop_timeout)
        if emergency_active:
            self.output = (0.0, 0.0, 0.0)
            if not self.emergency_stop_applied:
                self.publish_stop()
                self.emergency_stop_applied = True
                self.get_logger().warn(
                    'LIVE OBSTACLE emergency stop; navigation remains armed')
            else:
                self.publish_move(0.0, 0.0, 0.0)
            self.reason = '实时障碍进入紧急刹车距离，原地等待重新规划'
            return
        self.emergency_stop_applied = False
        if self.teleop_enabled:
            selected_cmd = self.teleop_cmd
            selected_cmd_t = self.teleop_cmd_t
            selected_timeout = self.teleop_timeout
            missing_reason = '启用后未收到键盘控制指令'
            stale_reason = '键盘控制指令超时'
            command_epoch_t = self.armed_t
            initial_timeout = 1.0
        else:
            if self.cancelled:
                return
            # Recovery is a separate, tightly scoped command source.  It may
            # override SCAN only while both producers agree that SCAN is in
            # WAIT_REPLAN. Any topic loss falls back to the normal watchdog.
            if (self.navigation_backend == 'scan' and
                    self.recovery_active and self.local_waiting):
                selected_cmd = self.recovery_cmd
                selected_cmd_t = self.recovery_cmd_t
                selected_timeout = self.cmd_timeout
                missing_reason = '脱困已激活但未收到脱困速度指令'
                stale_reason = '脱困速度指令超时'
            else:
                selected_cmd = self.planner_cmds[self.navigation_backend]
                selected_cmd_t = self.planner_cmd_times[
                    self.navigation_backend]
                selected_timeout = self.cmd_timeout
                missing_reason = '启用后未收到新的%s指令' % (
                    self.navigation_backend.upper())
                stale_reason = '%s速度指令超时' % (
                    self.navigation_backend.upper())
            command_epoch_t = max(
                stamp for stamp in
                (self.armed_t, self.navigation_started_t)
                if stamp is not None)
            initial_timeout = self.initial_cmd_timeout
        if not planner_command_ready(selected_cmd_t, command_epoch_t):
            self.output = (0.0, 0.0, 0.0)
            if planner_command_start_timed_out(
                    selected_cmd_t, command_epoch_t, now, initial_timeout):
                self.trip(missing_reason)
            return
        if now - selected_cmd_t > selected_timeout:
            self.trip(stale_reason)
            return

        vx = self.slew_with_braking(
            self.output[0], selected_cmd[0], self.max_accel,
            self.max_decel, dt)
        vy = self.slew_with_braking(
            self.output[1], selected_cmd[1], self.max_accel,
            self.max_decel, dt)
        vyaw = self.slew_with_braking(
            self.output[2], selected_cmd[2], self.max_yaw_accel,
            self.max_yaw_decel, dt)
        self.output = (vx, vy, vyaw)
        self.publish_move(vx, vy, vyaw)
        moving = any(abs(value) > 1e-3 for value in self.output)
        if self.teleop_enabled:
            self.reason = '底盘正在执行键盘控制' if moving else '键盘控制已启用，等待按键'
        elif (self.navigation_backend == 'scan' and
              self.recovery_active and self.local_waiting):
            self.reason = '底盘正在执行实时点云脱困' if moving else '实时点云脱困已激活，保持零速'
        else:
            self.reason = '底盘正在执行导航' if moving else '底盘已启用，等待运动指令'

    def publish_status(self):
        now = time.monotonic()
        fault = self.health_fault(now)
        navigation_odom_age = self.navigation_odom_age(now)
        selected_planner_t = self.planner_cmd_times[self.navigation_backend]
        status = {
            'connected': self.request_pub.get_subscription_count() > 0,
            'enabled': self.armed,
            'cancelled': self.cancelled,
            'cancel_seq': self.cancel_seq,
            'posture_seq': self.posture_seq,
            'posture_state': self.posture_state,
            'posture_pending': self.pending_posture is not None,
            'posture_busy': (
                self.pending_posture is not None or
                self.pending_sport_request is not None or
                self.posture_state in
                ('preparing', 'locking_for_lie', 'lying_down',
                 'standing_locked', 'recovering')),
            'sport_request_pending': self.pending_sport_request is not None,
            'posture_retry_count': (
                0 if self.pending_sport_request is None else
                self.pending_sport_request.get('rejection_count', 0)),
            'last_sport_response_code': self.last_sport_response_code,
            'posture_recovery_remaining': (
                None if self.posture_stage_deadline is None else
                round(max(0.0, self.posture_stage_deadline - now), 1)),
            'sport_mode': self.sport_mode,
            'sport_progress': self.sport_progress,
            'sport_state_age': (
                None if self.sport_state_t is None else
                round(now - self.sport_state_t, 3)),
            'control_mode': ('teleop' if self.teleop_enabled else
                             ('recovery' if self.navigation_backend == 'scan' and
                              self.recovery_active and
                              self.local_waiting else 'navigation')),
            'navigation_backend': self.navigation_backend,
            'backend_switch_error': self.backend_switch_error,
            'teleop_enabled': self.teleop_enabled,
            'ready': not bool(fault),
            'reason': self.reason if (
                self.posture_state != 'movable' or not fault or self.armed
            ) else fault,
            'cmd_age': None if (
                self.teleop_cmd_t if self.teleop_enabled else selected_planner_t
            ) is None else round(now - (
                self.teleop_cmd_t if self.teleop_enabled else selected_planner_t), 3),
            'planner_cmd_age': None if selected_planner_t is None else round(
                now - selected_planner_t, 3),
            'scan_cmd_age': None if self.planner_cmd_times['scan'] is None else round(
                now - self.planner_cmd_times['scan'], 3),
            'nav2_cmd_age': None if self.planner_cmd_times['nav2'] is None else round(
                now - self.planner_cmd_times['nav2'], 3),
            'teleop_cmd_age': None if self.teleop_cmd_t is None else round(
                now - self.teleop_cmd_t, 3),
            'recovery_active': bool(
                self.navigation_backend == 'scan' and self.recovery_active and
                self.local_waiting),
            'recovery_cmd_age': None if self.recovery_cmd_t is None else round(
                now - self.recovery_cmd_t, 3),
            'odom_age': None if navigation_odom_age is None else round(
                navigation_odom_age, 3),
            'body_odom_age': None if self.odom_t is None else round(
                now - self.odom_t, 3),
            'raw_odom_age': None if self.raw_odom_t is None else round(
                now - self.raw_odom_t, 3),
            'lowstate_age': None if self.lowstate_t is None else round(now - self.lowstate_t, 3),
            'heartbeat_age': None if self.heartbeat_t is None else round(now - self.heartbeat_t, 3),
            'alignment_valid': self.alignment_valid,
            'navigation_start_age': (
                None if self.navigation_started_t is None else
                round(now - self.navigation_started_t, 3)),
            'output': [round(value, 3) for value in self.output],
            'limits': [self.max_vx, self.max_vy, self.max_vyaw],
            'teleop_limits': [self.teleop_max_vx, self.max_vy, self.max_vyaw],
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(message)
        self.publish_enabled()

    def shutdown(self):
        if self.ever_armed:
            for _ in range(5):
                self.publish_stop()
                time.sleep(0.03)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
