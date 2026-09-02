import json
import os
import tempfile
import unittest

from go2_remembr.service import RemembrService, UnavailableRemembrService


class FakeCamera:
    def __init__(self):
        self.starts = 0

    def start(self):
        self.starts += 1
        return True, 'ok'

    def sample(self):
        return None


class FakeNavigation:
    def __init__(self):
        self.plan_calls = 0
        self.motion_commands = 0
        self.pose = {
            'map_signature': 'map-a', 'map_source': 'map.npz',
            'navigation_session_id': 'session-a',
            'x': 0.0, 'y': 0.0, 'z': 0.1, 'yaw': 0.0,
            'observed_at': 100.0,
        }

    def memory_pose_snapshot(self):
        return dict(self.pose), ''

    def memory_map_snapshot(self):
        return {
            'map_signature': self.pose['map_signature'],
            'map_source': self.pose['map_source'],
            'navigation_session_id': self.pose['navigation_session_id'],
        }, ''

    def preview_plan(self, start_x, start_y, goal_x, goal_y):
        self.plan_calls += 1
        return True, 'safe preview', {
            'start': {'x': start_x, 'y': start_y},
            'goal': {'x': goal_x, 'y': goal_y},
            'path': [start_x, start_y, 0.1, goal_x, goal_y, 0.1],
            'distance': 1.0, 'keypoints': 2,
        }


class ServiceSafetyTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.config_path = os.path.join(self.tempdir.name, 'config.json')
        with open(self.config_path, 'w', encoding='utf-8') as stream:
            json.dump({
                'database_path': os.path.join(self.tempdir.name, 'memory.sqlite3'),
                'vlm': {'backend': 'disabled'},
            }, stream)
        self.navigation = FakeNavigation()
        self.camera = FakeCamera()
        self.service = RemembrService(
            self.camera, self.navigation, self.config_path)

    def tearDown(self):
        self.service.shutdown()
        self.tempdir.cleanup()

    def test_vlm_disabled_but_manual_memory_and_query_work(self):
        enabled, message = self.service.set_enabled(True)
        self.assertFalse(enabled)
        self.assertIn('VLM', message)
        self.assertEqual(self.camera.starts, 0)

        ok, _, memory_id = self.service.add_manual('红色灭火器旁的走廊')
        self.assertTrue(ok)
        result = self.service.query({'mode': 'text', 'text': '灭火器'})

        self.assertTrue(result['success'])
        self.assertEqual(result['candidate']['memory_id'], memory_id)
        self.assertTrue(result['candidate']['requires_confirmation'])
        self.assertEqual(self.navigation.plan_calls, 1)
        self.assertEqual(self.navigation.motion_commands, 0)

    def test_manual_memory_at_similar_pose_keeps_only_latest(self):
        ok, _, old_id = self.service.add_manual('旧的门口描述')
        self.navigation.pose.update({
            'x': 0.18, 'y': 0.08, 'yaw': 0.12, 'observed_at': 110.0,
        })
        ok_latest, _, latest_id = self.service.add_manual('最新的门口描述')

        result = self.service.query({'mode': 'text', 'text': '门口'})

        self.assertTrue(ok)
        self.assertTrue(ok_latest)
        self.assertNotEqual(old_id, latest_id)
        self.assertEqual(self.service.store.count('map-a'), 1)
        self.assertEqual(result['memories'][0]['id'], latest_id)
        self.assertEqual(result['memories'][0]['caption'], '最新的门口描述')

    def test_execute_flag_is_rejected_before_planning(self):
        result = self.service.query({
            'mode': 'text', 'text': '任意地点', 'execute': True,
        })
        self.assertFalse(result['success'])
        self.assertIsNone(result['candidate'])
        self.assertEqual(self.navigation.plan_calls, 0)
        self.assertEqual(self.navigation.motion_commands, 0)

    def test_unavailable_service_never_breaks_into_motion(self):
        service = UnavailableRemembrService('database unavailable')
        result = service.query({'text': '门口'})

        self.assertFalse(service.status()['ready'])
        self.assertFalse(result['success'])
        self.assertIsNone(result['candidate'])
        self.assertEqual(self.navigation.motion_commands, 0)


if __name__ == '__main__':
    unittest.main()
