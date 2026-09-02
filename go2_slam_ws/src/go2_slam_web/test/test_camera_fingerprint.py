import unittest
from unittest import mock

import numpy as np

from go2_slam_web.navigation import CameraBridge, cv2


class CameraFingerprintTest(unittest.TestCase):
    def test_browser_image_path_does_not_compute_fingerprint(self):
        bridge = CameraBridge()
        with mock.patch.object(
                bridge, 'sample', return_value={'data': b'jpeg'}) as sample:
            self.assertEqual(bridge.image(), b'jpeg')
        sample.assert_called_once_with(include_fingerprint=False)

    @unittest.skipIf(cv2 is None, 'OpenCV is unavailable')
    def test_fingerprint_is_stable_for_same_jpeg(self):
        image = np.arange(9 * 16, dtype=np.uint8).reshape(9, 16)
        ok, encoded = cv2.imencode('.jpg', image)
        self.assertTrue(ok)

        first = CameraBridge._fingerprint(encoded.tobytes())
        second = CameraBridge._fingerprint(encoded.tobytes())

        self.assertIsInstance(first, int)
        self.assertEqual(first, second)
        self.assertGreaterEqual(first, 0)
        self.assertLess(first, 1 << 64)


if __name__ == '__main__':
    unittest.main()
