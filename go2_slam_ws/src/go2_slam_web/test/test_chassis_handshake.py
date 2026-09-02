import unittest

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


if __name__ == '__main__':
    unittest.main()
