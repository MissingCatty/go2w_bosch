import json
import math
import os
import tempfile
import time
import unittest

from go2_remembr.backends import HashEmbeddingBackend
from go2_remembr.store import MemoryStore


class MemoryStoreTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        embedding = HashEmbeddingBackend({
            'dimensions': 256, 'model': 'hash-char-v1',
        })
        self.store = MemoryStore(
            os.path.join(self.tempdir.name, 'memory.sqlite3'), embedding,
            deduplication={
                'enabled': True,
                'position_radius_m': 0.35,
                'z_tolerance_m': 0.30,
                'yaw_tolerance_deg': 20.0,
            })
        self.pose = {
            'map_signature': 'map-a', 'map_source': 'map.npz',
            'navigation_session_id': 'session-a',
            'x': 1.0, 'y': 2.0, 'z': 0.1, 'yaw': 0.2,
        }

    def tearDown(self):
        self.store.close()
        self.tempdir.cleanup()

    def test_text_search_and_map_isolation(self):
        wanted = self.store.add('走廊尽头有一个红色灭火器', self.pose)
        other_pose = dict(self.pose, x=8.0, y=9.0)
        self.store.add('大厅入口旁边是蓝色沙发', other_pose)
        foreign_pose = dict(self.pose, map_signature='map-b')
        self.store.add('红色灭火器在门边', foreign_pose)

        results = self.store.search_text('红色灭火器', 'map-a', limit=5)

        self.assertEqual(results[0]['id'], wanted)
        self.assertEqual(len(results), 2)
        self.assertEqual(self.store.count('map-a'), 2)
        self.assertEqual(self.store.count(), 3)

    def test_position_and_time_search(self):
        now = time.time()
        near_id = self.store.add('近处', self.pose, observed_at=now - 5)
        far_pose = dict(self.pose, x=5.0, y=5.0)
        self.store.add('远处', far_pose, observed_at=now)

        positioned = self.store.search_position(
            1.1, 2.1, 'map-a', limit=5, radius=1.0)
        timed = self.store.search_time(
            'map-a', limit=5, since=now - 10, until=now - 1)

        self.assertEqual([item['id'] for item in positioned], [near_id])
        self.assertEqual([item['id'] for item in timed], [near_id])

    def test_similar_pose_is_atomically_replaced_by_latest_memory(self):
        old_id = self.store.add('旧的走廊描述', self.pose)
        latest_pose = dict(
            self.pose, x=1.20, y=2.10, z=0.20,
            yaw=self.pose['yaw'] + math.radians(12.0),
            navigation_session_id='session-b')
        latest_id = self.store.add('最新的走廊描述', latest_pose)

        memories = self.store.search_time('map-a', limit=5)

        self.assertEqual(self.store.count('map-a'), 1)
        self.assertEqual([item['id'] for item in memories], [latest_id])
        self.assertEqual(memories[0]['caption'], '最新的走廊描述')
        self.assertNotEqual(old_id, latest_id)

    def test_pose_deduplication_respects_heading_height_and_map(self):
        self.store.add('基准视角', self.pose)
        self.store.add('反向视角', dict(
            self.pose, yaw=self.pose['yaw'] + math.radians(45.0)))
        self.store.add('不同高度', dict(self.pose, z=0.60))
        self.store.add('另一张地图', dict(
            self.pose, map_signature='map-b'))

        self.assertEqual(self.store.count('map-a'), 3)
        self.assertEqual(self.store.count('map-b'), 1)

    def test_pose_deduplication_handles_yaw_wraparound(self):
        first = dict(self.pose, yaw=math.pi - math.radians(4.0))
        latest = dict(self.pose, yaw=-math.pi + math.radians(4.0))
        self.store.add('跨过正180度前', first)
        latest_id = self.store.add('跨过正180度后', latest)

        memories = self.store.search_time('map-a', limit=5)
        self.assertEqual(self.store.count('map-a'), 1)
        self.assertEqual(memories[0]['id'], latest_id)

    def test_failed_replacement_rolls_back_and_keeps_old_memory(self):
        old_id = self.store.add('不能丢失的旧记忆', self.pose)

        with self.assertRaises((TypeError, ValueError)):
            self.store.add('写入会失败的新记忆', self.pose,
                           frame_count='not-an-integer')

        memories = self.store.search_time('map-a', limit=5)
        self.assertEqual(self.store.count('map-a'), 1)
        self.assertEqual(memories[0]['id'], old_id)
        self.assertEqual(memories[0]['caption'], '不能丢失的旧记忆')

    def test_unchanged_keyframe_refreshes_latest_pose_without_embedding(self):
        now = time.time()
        memory_id = self.store.add(
            '二维码旁的玻璃门', self.pose, frame_count=2,
            observed_at=now - 2,
            metadata={
                'perceptual_fingerprint': 0b1010,
                'last_vlm_at': now - 2,
            })
        latest_pose = dict(
            self.pose, x=1.15, y=2.05,
            navigation_session_id='session-b')

        refreshed_id = self.store.refresh_similar_if_unchanged(
            latest_pose, 0b1011, max_hamming=4, max_vlm_age_s=30,
            frame_count=1, observed_at=now,
            metadata={'vlm_skipped': True})

        self.assertEqual(refreshed_id, memory_id)
        memories = self.store.search_time('map-a', limit=5)
        self.assertEqual(len(memories), 1)
        self.assertEqual(memories[0]['caption'], '二维码旁的玻璃门')
        self.assertEqual(memories[0]['frame_count'], 1)
        self.assertAlmostEqual(memories[0]['x'], 1.15)
        with self.store.lock:
            row = self.store.connection.execute(
                'SELECT metadata_json FROM memories WHERE id = ?',
                (memory_id,)).fetchone()
        metadata = json.loads(row['metadata_json'])
        self.assertTrue(metadata['vlm_skipped'])
        self.assertEqual(metadata['keyframe_refresh_count'], 1)
        self.assertEqual(metadata['last_vlm_at'], now - 2)

    def test_keyframe_refresh_rejects_changed_or_stale_image(self):
        now = time.time()
        memory_id = self.store.add(
            '原始场景', self.pose,
            metadata={
                'perceptual_fingerprint': 0,
                'last_vlm_at': now - 60,
            })

        changed = self.store.refresh_similar_if_unchanged(
            self.pose, (1 << 64) - 1, max_hamming=4,
            max_vlm_age_s=120)
        stale = self.store.refresh_similar_if_unchanged(
            self.pose, 0, max_hamming=4, max_vlm_age_s=30)

        self.assertIsNone(changed)
        self.assertIsNone(stale)
        self.assertEqual(self.store.search_time('map-a', 1)[0]['id'], memory_id)


if __name__ == '__main__':
    unittest.main()
