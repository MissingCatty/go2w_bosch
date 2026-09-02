#!/usr/bin/env python3
"""Bridge simple Web goals/cancels to Nav2's NavigateToPose action."""

import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class NavGoalBridge(Node):
    def __init__(self):
        super().__init__('go2_nav2_goal_bridge')
        self.declare_parameter('goal_topic', '/go2/nav2/goal')
        self.declare_parameter('cancel_topic', '/go2/nav2/cancel')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('server_wait_timeout', 0.2)

        self.client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter('action_name').value))
        self.status_pub = self.create_publisher(String, '/go2/nav2/status', 10)
        self.completed_pub = self.create_publisher(
            Bool, '/go2/nav2/navigation_completed', 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value),
            self.on_goal, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('cancel_topic').value),
            self.on_cancel, 10)

        self.pending_goal = None
        self.goal_handle = None
        self.goal_seq = 0
        self.state = 'waiting_for_server'
        self.message = '等待 Nav2 NavigateToPose action'
        self.updated_t = time.monotonic()
        self.feedback = {}
        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self.publish_status)

    def set_state(self, state, message):
        self.state = state
        self.message = str(message)
        self.updated_t = time.monotonic()

    def on_goal(self, message):
        if message.header.frame_id != 'nav_map':
            self.set_state(
                'rejected', '目标坐标系必须是 nav_map，收到 %s' %
                message.header.frame_id)
            return
        self.pending_goal = message
        self.goal_seq += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.set_state('queued', '新目标已排队，等待 Nav2 action server')

    def on_cancel(self, message):
        if not message.data:
            return
        self.pending_goal = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.set_state('canceled', '已请求取消 Nav2 导航')

    def tick(self):
        if self.pending_goal is None:
            return
        wait_timeout = max(
            0.0, float(self.get_parameter('server_wait_timeout').value))
        if not self.client.wait_for_server(timeout_sec=wait_timeout):
            self.set_state('waiting_for_server', 'Nav2 action server 尚未就绪')
            return
        pose = self.pending_goal
        self.pending_goal = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        sequence = self.goal_seq
        future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        future.add_done_callback(
            lambda result, seq=sequence: self.on_goal_response(result, seq))
        self.set_state('sending', '正在向 Nav2 发送目标')

    def on_goal_response(self, future, sequence):
        if sequence != self.goal_seq:
            return
        try:
            handle = future.result()
        except Exception as exc:  # rclpy transport/action error
            self.set_state('error', 'Nav2 目标发送失败: %s' % exc)
            return
        if not handle.accepted:
            self.set_state('rejected', 'Nav2 拒绝目标')
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda value, seq=sequence: self.on_result(value, seq))
        self.set_state('executing', 'Nav2 已接受目标')

    def on_feedback(self, message):
        feedback = message.feedback
        self.feedback = {
            'distance_remaining': round(
                float(getattr(feedback, 'distance_remaining', 0.0)), 3),
            'recoveries': int(getattr(feedback, 'number_of_recoveries', 0)),
        }
        if self.state != 'executing':
            self.set_state('executing', 'Nav2 正在执行目标')

    def on_result(self, future, sequence):
        if sequence != self.goal_seq:
            return
        self.goal_handle = None
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self.set_state('error', '读取 Nav2 结果失败: %s' % exc)
            return
        name = STATUS_NAMES.get(status, 'status_%d' % status)
        self.set_state(name, 'Nav2 导航结果: %s' % name)
        completed = Bool()
        completed.data = status == GoalStatus.STATUS_SUCCEEDED
        self.completed_pub.publish(completed)

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'message': self.message,
            'goal_seq': self.goal_seq,
            'active': self.goal_handle is not None,
            'age': round(time.monotonic() - self.updated_t, 3),
            'feedback': self.feedback,
        }, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
