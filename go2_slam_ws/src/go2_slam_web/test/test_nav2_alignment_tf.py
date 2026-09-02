import math
import threading
import unittest

from go2_slam_web.navigation import NavigationState


class Nav2AlignmentTransformTest(unittest.TestCase):

    def make_navigation(self, transform, valid=True):
        navigation = NavigationState.__new__(NavigationState)
        navigation.lock = threading.Lock()
        navigation.alignment_valid = valid
        navigation.map_to_odom = transform
        return navigation

    def test_invalid_alignment_publishes_no_tf(self):
        navigation = self.make_navigation((1.0, 2.0, 0.3), valid=False)
        self.assertIsNone(navigation.nav2_map_to_odom_tf())

    def test_nav_map_to_odom_is_historical_transform_inverse(self):
        historical = (1.2, -0.7, math.radians(32.0))
        navigation = self.make_navigation(historical)
        transform = navigation.nav2_map_to_odom_tf()

        # Compose inverse(nav_map->odom) with the historical map->odom point
        # conversion. Every input map point must return unchanged.
        for point in ((0.0, 0.0), (2.3, -1.1), (-4.0, 3.5)):
            odom = NavigationState._map_to_odom_xy(*point, historical)
            c = math.cos(transform['yaw'])
            s = math.sin(transform['yaw'])
            recovered = (
                c * odom[0] - s * odom[1] + transform['x'],
                s * odom[0] + c * odom[1] + transform['y'],
            )
            self.assertAlmostEqual(recovered[0], point[0], places=7)
            self.assertAlmostEqual(recovered[1], point[1], places=7)


if __name__ == '__main__':
    unittest.main()
