import json
import os
import tempfile
import threading
import time
import unittest

from go2_remembr.service import RemembrService


class StreamingCamera:
    def __init__(self):
        self.counter = 0

    def start(self):
        return True, 'ok'

    def sample(self):
        self.counter += 1
        return {
            'token': self.counter,
            'data': ('frame-%d' % self.counter).encode('ascii'),
        }


class MovingNavigation:
    def __init__(self):
        self.pose_counter = 0

    def memory_pose_snapshot(self):
        self.pose_counter += 1
        return {
            'map_signature': 'map-a',
            'map_source': 'map.npz',
            'navigation_session_id': 'session-a',
            'x': self.pose_counter * 0.01,
            'y': 0.0,
            'z': 0.1,
            'yaw': 0.0,
            'observed_at': 100.0 + self.pose_counter * 0.25,
        }, ''

    def memory_map_snapshot(self):
        return {
            'map_signature': 'map-a',
            'map_source': 'map.npz',
            'navigation_session_id': 'session-a',
        }, ''


class BlockingVision:
    name = 'test_vlm'
    model = 'blocking-test-model'
    ready = True

    def __init__(self):
        self.calls = []
        self.started = threading.Event()
        self.release = threading.Event()

    def online(self):
        return True

    def caption(self, frames, prompt):
        self.calls.append(list(frames))
        if len(self.calls) == 1:
            self.started.set()
            if not self.release.wait(timeout=4.0):
                raise RuntimeError('test inference release timed out')
        return frames[-1].decode('ascii')


class LatestOnlyPipelineTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.config_path = os.path.join(self.tempdir.name, 'config.json')
        with open(self.config_path, 'w', encoding='utf-8') as stream:
            json.dump({
                'database_path': os.path.join(
                    self.tempdir.name, 'memory.sqlite3'),
                'capture': {
                    'sample_interval_s': 0.25,
                    'segment_duration_s': 0.5,
                    'frames_per_segment': 2,
                    'prompt': 'describe',
                },
                'vlm': {'backend': 'disabled'},
            }, stream)
        self.camera = StreamingCamera()
        self.navigation = MovingNavigation()
        self.vision = BlockingVision()
        self.service = RemembrService(
            self.camera, self.navigation, self.config_path)
        self.service.vision = self.vision

    def tearDown(self):
        self.vision.release.set()
        self.service.shutdown()
        self.tempdir.cleanup()

    @staticmethod
    def wait_for(predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.02)
        return None

    def test_busy_inference_keeps_only_latest_segment_and_bound_pose(self):
        enabled, _ = self.service.set_enabled(True)
        self.assertTrue(enabled)
        self.assertTrue(self.vision.started.wait(timeout=2.0))

        dropped = self.wait_for(
            lambda: self.service.status()['pipeline']['segments_dropped'] >= 1)
        self.assertTrue(dropped)
        busy_status = self.service.status()['pipeline']
        self.assertEqual(busy_status['pending_segments'], 1)
        self.assertTrue(busy_status['inference_active'])

        self.vision.release.set()
        processed = self.wait_for(
            lambda: self.service.status()['pipeline']['segments_processed'] >= 2)
        self.assertTrue(processed)

        status = self.service.status()
        pipeline = status['pipeline']
        memories = self.service.store.search_time('map-a', limit=5)
        latest = memories[0]
        with self.service.store.lock:
            metadata_row = self.service.store.connection.execute(
                'SELECT metadata_json FROM memories LIMIT 1').fetchone()
        metadata = json.loads(metadata_row['metadata_json'])
        frame_number = int(latest['caption'].split('-')[1])

        self.assertEqual(pipeline['queue_policy'], 'latest_only')
        self.assertGreaterEqual(pipeline['segments_dropped'], 1)
        self.assertLessEqual(pipeline['pending_segments'], 1)
        self.assertTrue(all(len(frames) == 2 for frames in self.vision.calls))
        self.assertGreater(
            int(self.vision.calls[1][-1].decode().split('-')[1]),
            int(self.vision.calls[0][-1].decode().split('-')[1]) + 1)
        self.assertEqual(latest['frame_count'], 2)
        self.assertEqual(metadata['queue_policy'], 'latest_only')
        self.assertAlmostEqual(metadata['capture_window_s'], 0.5, delta=0.03)
        # set_enabled() consumes one pose snapshot before frame-1, so the
        # latest image frame-N must retain pose N+1 from capture time.
        self.assertAlmostEqual(latest['x'], (frame_number + 1) * 0.01, places=3)

    def test_adaptive_frames_uses_only_latest_when_images_are_similar(self):
        self.service.adaptive_frames = True
        pose_a, _ = self.navigation.memory_pose_snapshot()
        pose_b, _ = self.navigation.memory_pose_snapshot()
        with self.service.condition:
            self.service._append_sample_locked(
                {'data': b'first', 'fingerprint': 0}, pose_a)
            self.service._append_sample_locked(
                {'data': b'latest', 'fingerprint': 0b11}, pose_b)
            self.service._enqueue_current_segment_locked(1, 123.0)
            segment = self.service.pending_segment
        self.assertEqual(segment['frames'], [b'latest'])
        self.assertEqual(segment['poses'], [pose_b])
        self.assertEqual(segment['captured_frame_count'], 2)
        self.assertTrue(segment['adaptive_single_frame'])
        self.assertEqual(segment['perceptual_hamming_distance'], 2)

    def test_adaptive_frames_keeps_two_when_images_changed(self):
        self.service.adaptive_frames = True
        pose_a, _ = self.navigation.memory_pose_snapshot()
        pose_b, _ = self.navigation.memory_pose_snapshot()
        with self.service.condition:
            self.service._append_sample_locked(
                {'data': b'first', 'fingerprint': 0}, pose_a)
            self.service._append_sample_locked(
                {'data': b'latest', 'fingerprint': (1 << 64) - 1}, pose_b)
            self.service._enqueue_current_segment_locked(1, 123.0)
            segment = self.service.pending_segment
        self.assertEqual(segment['frames'], [b'first', b'latest'])
        self.assertEqual(segment['poses'], [pose_a, pose_b])
        self.assertFalse(segment['adaptive_single_frame'])
        self.assertEqual(segment['perceptual_hamming_distance'], 64)

    def test_pose_and_image_keyframe_refresh_skips_vlm(self):
        self.service.keyframe_filter_enabled = True
        self.service.keyframe_filter_hamming = 4
        self.service.keyframe_filter_max_vlm_age = 30.0
        self.service.enabled = True
        self.service.capture_generation = 1
        pose, _ = self.navigation.memory_pose_snapshot()
        self.service.store.add(
            '已有的稳定场景描述', pose, observed_at=time.time() - 1,
            metadata={
                'perceptual_fingerprint': 0b1010,
                'last_vlm_at': time.time() - 1,
            })
        segment = {
            'generation': 1,
            'frames': [b'latest'],
            'poses': [pose],
            'captured_frame_count': 2,
            'segment_started_at': pose['observed_at'] - 0.5,
            'segment_ended_at': pose['observed_at'],
            'adaptive_single_frame': True,
            'perceptual_hamming_distance': 1,
            'perceptual_fingerprint': 0b1011,
            'queued_at': time.time(),
        }
        self.vision.release.set()

        outcome = self.service._process_segment(segment)

        self.assertEqual(outcome, 'refreshed')
        self.assertEqual(self.vision.calls, [])
        memories = self.service.store.search_time('map-a', limit=5)
        self.assertEqual(len(memories), 1)
        self.assertEqual(memories[0]['caption'], '已有的稳定场景描述')
        self.assertEqual(memories[0]['frame_count'], 1)


if __name__ == '__main__':
    unittest.main()
