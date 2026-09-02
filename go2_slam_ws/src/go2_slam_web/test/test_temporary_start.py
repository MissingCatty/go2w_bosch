import math
import unittest

import numpy as np
from scipy import ndimage

from go2_slam_web.navigation import NavigationState


class TemporaryStartTest(unittest.TestCase):

    def make_navigation(self):
        navigation = NavigationState.__new__(NavigationState)
        navigation.resolution = 0.10
        navigation.origin = (-1.0, -1.0)
        navigation.free = np.zeros((21, 21), dtype=np.bool_)
        # The robot is at map (0, 0), cell (10, 10), inside the saved
        # obstacle. A connected free half-plane begins 0.35 m to its right.
        navigation.free[:, 13:] = True
        navigation.clearance_m = ndimage.distance_transform_edt(
            navigation.free, sampling=navigation.resolution).astype(np.float32)
        navigation.free_components, _ = ndimage.label(
            navigation.free,
            structure=np.array(
                [[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8))
        return navigation

    def test_static_obstacle_alignment_is_allowed_when_exit_exists(self):
        navigation = self.make_navigation()
        valid, message = navigation.validate_alignment_pose(0.0, 0.0)
        self.assertTrue(valid)
        self.assertIn('临时起点', message)

    def test_temporary_start_is_outside_minimum_radius_and_live_obstacle(self):
        navigation = self.make_navigation()
        start = (0.0, 0.0)
        goal = navigation._to_cell(0.85, 0.05)
        baseline = navigation._select_temporary_start(
            start, goal, np.empty((0, 3), dtype=np.float32))
        self.assertIsNotNone(baseline)
        self.assertGreaterEqual(baseline['distance'], 0.30)
        self.assertTrue(
            navigation.free[baseline['cell'][1], baseline['cell'][0]])

        blocker = np.array(
            [[baseline['world'][0], baseline['world'][1], 0.4]],
            dtype=np.float32)
        diverted = navigation._select_temporary_start(start, goal, blocker)
        self.assertIsNotNone(diverted)
        self.assertNotEqual(diverted['cell'], baseline['cell'])
        self.assertGreater(
            math.hypot(diverted['world'][0] - blocker[0, 0],
                       diverted['world'][1] - blocker[0, 1]),
            navigation.resolution * 0.75)

    def test_no_temporary_start_when_live_layer_blocks_every_candidate(self):
        navigation = self.make_navigation()
        candidates = navigation._temporary_start_candidates((0.0, 0.0))
        self.assertIsNotNone(candidates)
        _, _, wx, wy, _ = candidates
        live = np.column_stack(
            (wx, wy, np.full(len(wx), 0.4))).astype(np.float32)
        selected = navigation._select_temporary_start(
            (0.0, 0.0), navigation._to_cell(0.85, 0.05), live)
        self.assertIsNone(selected)


if __name__ == '__main__':
    unittest.main()
