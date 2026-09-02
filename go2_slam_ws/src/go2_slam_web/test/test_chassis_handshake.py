import unittest

from go2_slam_web.chassis_safety_gate import (
    planner_command_ready,
    planner_command_start_timed_out,
)
from go2_slam_web.web_server import (
    chassis_cancel_acknowledged,
    chassis_cancel_is_safe,
)


class ChassisHandshakeTest(unittest.TestCase):

    def test_cancelled_locked_zero_output_is_safe(self):
        self.assertTrue(chassis_cancel_is_safe({
            'cancelled': True,
            'enabled': False,
            'output': [0.0, 0.0, 0.0],
        }))

    def test_armed_or_moving_state_is_not_safe(self):
        self.assertFalse(chassis_cancel_is_safe({
            'cancelled': True,
            'enabled': True,
            'output': [0.0, 0.0, 0.0],
        }))
        self.assertFalse(chassis_cancel_is_safe({
            'cancelled': True,
            'enabled': False,
            'output': [0.02, 0.0, 0.0],
        }))

    def test_incomplete_status_is_not_safe(self):
        self.assertFalse(chassis_cancel_is_safe({
            'cancelled': True,
            'enabled': False,
        }))

    def test_cancel_ack_requires_new_sequence_and_fresh_safe_state(self):
        status = {
            'cancelled': True,
            'enabled': False,
            'cancel_seq': 8,
            'output': [0.0, 0.0, 0.0],
        }
        self.assertFalse(chassis_cancel_acknowledged(status, 0.1, 8))
        self.assertTrue(chassis_cancel_acknowledged(status, 0.1, 7))
        self.assertFalse(chassis_cancel_acknowledged(status, 1.1, 7))


class PlannerCommandEpochTest(unittest.TestCase):

    def test_goal_epoch_starts_a_fresh_bounded_grace(self):
        goal_started = 20.0
        self.assertFalse(planner_command_start_timed_out(
            None, goal_started, 22.99, 3.0))
        self.assertTrue(planner_command_start_timed_out(
            None, goal_started, 23.01, 3.0))

    def test_stale_pre_goal_command_is_rejected(self):
        goal_started = 20.0
        self.assertFalse(planner_command_ready(19.99, goal_started))
        self.assertTrue(planner_command_ready(20.01, goal_started))
        self.assertFalse(planner_command_start_timed_out(
            20.01, goal_started, 30.0, 3.0))

    def test_missing_epoch_fails_closed(self):
        self.assertTrue(planner_command_start_timed_out(
            None, None, 10.0, 3.0))


if __name__ == '__main__':
    unittest.main()
