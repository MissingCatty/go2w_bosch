"""Edge capture, semantic indexing and safe navigation-candidate generation."""

import math
import threading
import time

from .backends import make_embedding_backend, make_reasoner, make_vision_backend
from .config import load_config
from .store import MemoryStore


class RemembrService:
    """Own semantic memory only; the existing navigation stack owns motion."""

    def __init__(self, camera, navigation, config_path=None):
        self.camera = camera
        self.navigation = navigation
        self.config, self.config_path = load_config(config_path)
        self.vision = make_vision_backend(self.config.get('vlm', {}))
        self.embedding = make_embedding_backend(self.config.get('embedding', {}))
        self.reasoner = make_reasoner(self.config.get('reasoner', {}))
        query = self.config.get('query', {})
        self.default_limit = max(1, int(query.get('default_limit', 5)))
        self.max_limit = max(self.default_limit, int(query.get('max_limit', 20)))
        self.store = MemoryStore(
            self.config['database_path'], self.embedding,
            max_scan=query.get('max_scan', 50000),
            deduplication=self.config.get('deduplication', {}))
        capture = self.config.get('capture', {})
        self.sample_interval = max(0.25, float(capture.get('sample_interval_s', 1.0)))
        self.segment_duration = max(
            self.sample_interval, float(capture.get('segment_duration_s', 10.0)))
        self.frames_per_segment = max(1, min(32, int(
            capture.get('frames_per_segment', 6))))
        self.adaptive_frames = bool(capture.get('adaptive_frames', False))
        self.adaptive_frame_hamming = max(0, min(64, int(
            capture.get('adaptive_frame_hamming', 6))))
        keyframe_filter = self.config.get('keyframe_filter', {})
        self.keyframe_filter_enabled = bool(
            keyframe_filter.get('enabled', False))
        self.keyframe_filter_hamming = max(0, min(64, int(
            keyframe_filter.get('image_hamming', 4))))
        self.keyframe_filter_max_vlm_age = max(0.0, float(
            keyframe_filter.get('max_vlm_age_s', 30.0)))
        self.max_caption_chars = max(64, int(capture.get('max_caption_chars', 3000)))
        self.prompt = str(capture.get('prompt', '')).strip()
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.stop_event = threading.Event()
        self.worker_stop_event = threading.Event()
        self.thread = None
        self.inference_thread = None
        self.capture_generation = 0
        self.enabled = False
        self.started_at = 0.0
        self.last_memory_at = 0.0
        self.last_error = ''
        self.segment_frames = []
        self.segment_poses = []
        self.segment_fingerprints = []
        self.segment_started_at = 0.0
        self.segment_started_monotonic = 0.0
        self.last_frame_token = None
        self.pending_segment = None
        self.inference_active = False
        self.last_inference_s = 0.0
        self.segments_completed = 0
        self.segments_processed = 0
        self.segments_dropped = 0
        self.single_frame_segments = 0
        self.multi_frame_segments = 0
        self.keyframe_refreshes = 0
        self.last_refresh_s = 0.0
        if bool(self.config.get('auto_start', False)):
            self.set_enabled(True)

    def _reset_segment_locked(self):
        self.segment_frames = []
        self.segment_poses = []
        self.segment_fingerprints = []
        self.segment_started_at = 0.0
        self.segment_started_monotonic = 0.0

    def _append_sample_locked(self, sample, pose):
        """Keep the first and most recent samples in a bounded segment."""
        frame = sample['data']
        fingerprint = sample.get('fingerprint')
        pose = dict(pose)
        if len(self.segment_frames) < self.frames_per_segment:
            self.segment_frames.append(frame)
            self.segment_poses.append(pose)
            self.segment_fingerprints.append(fingerprint)
        elif self.frames_per_segment == 1:
            self.segment_frames[0] = frame
            self.segment_poses[0] = pose
            self.segment_fingerprints[0] = fingerprint
        else:
            # Preserve the segment's first observation and roll the remaining
            # bounded slots forward.  For the deployed two-frame mode this is
            # exactly the segment's first and latest image/pose pair.
            del self.segment_frames[1]
            del self.segment_poses[1]
            del self.segment_fingerprints[1]
            self.segment_frames.append(frame)
            self.segment_poses.append(pose)
            self.segment_fingerprints.append(fingerprint)

    def _start_segment_locked(self, sample, pose, wall_time, monotonic_time):
        self.segment_started_at = wall_time
        self.segment_started_monotonic = monotonic_time
        self._append_sample_locked(sample, pose)

    def _enqueue_current_segment_locked(self, generation, queued_at):
        if not self.segment_frames or not self.segment_poses:
            return
        frames = list(self.segment_frames)
        poses = [dict(pose) for pose in self.segment_poses]
        hamming_distance = None
        adaptive_single_frame = False
        if (self.adaptive_frames and len(frames) > 1 and
                len(self.segment_fingerprints) == len(frames)):
            first_hash = self.segment_fingerprints[0]
            latest_hash = self.segment_fingerprints[-1]
            if isinstance(first_hash, int) and isinstance(latest_hash, int):
                hamming_distance = bin(first_hash ^ latest_hash).count('1')
                if hamming_distance <= self.adaptive_frame_hamming:
                    frames = [frames[-1]]
                    poses = [poses[-1]]
                    adaptive_single_frame = True
        segment = {
            'generation': generation,
            'frames': frames,
            'poses': poses,
            'captured_frame_count': len(self.segment_frames),
            'segment_started_at': self.segment_poses[0]['observed_at'],
            'segment_ended_at': self.segment_poses[-1]['observed_at'],
            'adaptive_single_frame': adaptive_single_frame,
            'perceptual_hamming_distance': hamming_distance,
            'perceptual_fingerprint': self.segment_fingerprints[-1],
            'queued_at': queued_at,
        }
        self.segments_completed += 1
        if self.pending_segment is not None:
            self.segments_dropped += 1
        # A single overwrite slot provides backpressure without an unbounded
        # queue: when inference is busy, only the newest complete segment wins.
        self.pending_segment = segment
        self.condition.notify_all()

    def _ensure_inference_thread_locked(self):
        if self.inference_thread is not None and self.inference_thread.is_alive():
            return
        if self.worker_stop_event.is_set():
            raise RuntimeError('语义记忆推理线程已停止')
        self.inference_thread = threading.Thread(
            target=self._inference_loop,
            name='go2-remembr-inference', daemon=True)
        self.inference_thread.start()

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled and not self.vision.ready:
            return False, 'VLM 尚未配置，自动记忆保持关闭；仍可手工写入记忆'
        if enabled:
            if not self.vision.online():
                return False, 'VLM 已配置但本机推理服务尚未就绪'
            pose, reason = self.navigation.memory_pose_snapshot()
            if pose is None:
                return False, reason
            ok, message = self.camera.start()
            if not ok:
                return False, message
            with self.condition:
                if self.enabled:
                    return True, '语义记忆采集已开启'
                self.enabled = True
                self.capture_generation += 1
                generation = self.capture_generation
                self.started_at = time.time()
                self.last_error = ''
                self.stop_event = threading.Event()
                stop_event = self.stop_event
                self._reset_segment_locked()
                self.last_frame_token = None
                self.pending_segment = None
                self.last_inference_s = 0.0
                self.segments_completed = 0
                self.segments_processed = 0
                self.segments_dropped = 0
                self.single_frame_segments = 0
                self.multi_frame_segments = 0
                self.keyframe_refreshes = 0
                self.last_refresh_s = 0.0
                self._ensure_inference_thread_locked()
                self.thread = threading.Thread(
                    target=self._capture_loop, args=(stop_event, generation),
                    name='go2-remembr-capture', daemon=True)
                self.thread.start()
            return True, '语义记忆采集已开启'
        with self.condition:
            self.enabled = False
            self.capture_generation += 1
            self.stop_event.set()
            thread = self.thread
            self.thread = None
            self._reset_segment_locked()
            self.last_frame_token = None
            self.pending_segment = None
            self.condition.notify_all()
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        return True, '语义记忆采集已关闭'

    def _capture_loop(self, stop_event, generation):
        next_sample = time.monotonic()
        while not stop_event.is_set():
            delay = max(0.0, next_sample - time.monotonic())
            if stop_event.wait(delay):
                break
            try:
                self._capture_once(generation)
            except Exception as exc:
                with self.lock:
                    if self.enabled and generation == self.capture_generation:
                        self.last_error = str(exc)[:500]
            next_sample += self.sample_interval
            # Never replay missed sampling ticks after a scheduler stall.
            if next_sample < time.monotonic():
                next_sample = time.monotonic() + self.sample_interval

    def _capture_once(self, generation):
        with self.lock:
            if not self.enabled or generation != self.capture_generation:
                return
        pose, reason = self.navigation.memory_pose_snapshot()
        if pose is None:
            with self.condition:
                if self.enabled and generation == self.capture_generation:
                    self.last_error = reason
                    self._reset_segment_locked()
                    if self.pending_segment is not None:
                        self.pending_segment = None
                        self.segments_dropped += 1
            return
        sample = self.camera.sample()
        if sample is None:
            with self.lock:
                if self.enabled and generation == self.capture_generation:
                    self.last_error = '摄像头画面未就绪或已过期'
            return
        now = time.time()
        monotonic_now = time.monotonic()
        with self.condition:
            if (not self.enabled or generation != self.capture_generation or
                    sample['token'] == self.last_frame_token):
                return
            if self.segment_poses:
                first = self.segment_poses[0]
                if (first['map_signature'] != pose['map_signature'] or
                        first['navigation_session_id'] !=
                        pose['navigation_session_id']):
                    self._reset_segment_locked()
            self.last_frame_token = sample['token']
            if not self.segment_started_monotonic:
                self._start_segment_locked(
                    sample, pose, now, monotonic_now)
                return
            self._append_sample_locked(sample, pose)
            # Sampling wakeups can arrive a few microseconds before the exact
            # deadline.  Without a small tolerance a nominal 3.0 s segment at
            # 1.5 s/sample can accidentally wait for the 4.5 s tick.
            close_tolerance = min(0.05, self.sample_interval * 0.1)
            if (monotonic_now + close_tolerance -
                    self.segment_started_monotonic < self.segment_duration):
                return
            self._enqueue_current_segment_locked(generation, now)
            self._reset_segment_locked()
            # Reuse the boundary observation as the first sample of the next
            # continuous segment.  This avoids a sample-interval-sized hole.
            self._start_segment_locked(sample, pose, now, monotonic_now)

    def _inference_loop(self):
        while True:
            with self.condition:
                while (self.pending_segment is None and
                       not self.worker_stop_event.is_set()):
                    self.condition.wait(timeout=1.0)
                if self.worker_stop_event.is_set():
                    return
                segment = self.pending_segment
                self.pending_segment = None
                self.inference_active = True
            started = time.monotonic()
            try:
                outcome = self._process_segment(segment)
            except Exception as exc:
                with self.lock:
                    if (self.enabled and
                            segment['generation'] == self.capture_generation):
                        self.last_error = str(exc)[:500]
                outcome = None
            elapsed = time.monotonic() - started
            with self.condition:
                if outcome == 'refreshed':
                    self.last_refresh_s = elapsed
                    self.keyframe_refreshes += 1
                elif outcome == 'inferred':
                    self.last_inference_s = elapsed
                if outcome:
                    self.segments_processed += 1
                    if len(segment['frames']) == 1:
                        self.single_frame_segments += 1
                    else:
                        self.multi_frame_segments += 1
                self.inference_active = False
                self.condition.notify_all()

    def _process_segment(self, segment):
        frames = segment['frames']
        poses = segment['poses']
        if not frames or not poses:
            return None
        with self.lock:
            if (not self.enabled or
                    segment['generation'] != self.capture_generation):
                return None
        identity, reason = self.navigation.memory_map_snapshot()
        first = poses[0]
        if identity is None:
            raise ValueError(reason or '当前地图不可用于写入语义记忆')
        if (identity['map_signature'] != first['map_signature'] or
                identity['navigation_session_id'] !=
                first['navigation_session_id']):
            raise ValueError('地图或定位会话已变化，已丢弃旧视觉片段')
        middle = poses[len(poses) // 2]
        observed = sum(item['observed_at'] for item in poses) / len(poses)
        segment_started_at = segment.get(
            'segment_started_at', poses[0]['observed_at'])
        segment_ended_at = segment.get(
            'segment_ended_at', poses[-1]['observed_at'])
        base_metadata = {
            'vlm_backend': self.vision.name,
            'vlm_model': self.vision.model,
            'segment_started_at': segment_started_at,
            'segment_ended_at': segment_ended_at,
            'capture_window_s': max(
                0.0, segment_ended_at - segment_started_at),
            'captured_frame_count': segment.get(
                'captured_frame_count', len(frames)),
            'adaptive_single_frame': bool(segment.get(
                'adaptive_single_frame', False)),
            'perceptual_hamming_distance': segment.get(
                'perceptual_hamming_distance'),
            'perceptual_fingerprint': segment.get('perceptual_fingerprint'),
            'queue_policy': 'latest_only',
            'images_persisted': False,
        }
        if self.keyframe_filter_enabled:
            refresh_started_at = time.time()
            refresh_metadata = dict(base_metadata)
            refresh_metadata.update({
                'queue_delay_s': max(
                    0.0, refresh_started_at - segment['queued_at']),
                'vlm_inference_s': 0.0,
                'vlm_skipped': True,
                'keyframe_filter': 'pose_and_perceptual_hash',
            })
            refreshed_id = self.store.refresh_similar_if_unchanged(
                middle, segment.get('perceptual_fingerprint'),
                self.keyframe_filter_hamming,
                self.keyframe_filter_max_vlm_age,
                frame_count=len(frames), metadata=refresh_metadata,
                observed_at=observed)
            if refreshed_id:
                with self.lock:
                    self.last_memory_at = time.time()
                    self.last_error = ''
                return 'refreshed'

        inference_started_at = time.time()
        caption = self.vision.caption(frames, self.prompt).strip()
        if not caption:
            raise ValueError('VLM 返回了空描述')
        caption = caption[:self.max_caption_chars]
        with self.lock:
            if (not self.enabled or
                    segment['generation'] != self.capture_generation):
                return None
        identity, reason = self.navigation.memory_map_snapshot()
        if identity is None:
            raise ValueError(reason or '当前地图不可用于写入语义记忆')
        if (identity['map_signature'] != first['map_signature'] or
                identity['navigation_session_id'] !=
                first['navigation_session_id']):
            raise ValueError('地图或定位会话已变化，已丢弃旧视觉片段')
        inference_finished_at = time.time()
        metadata = dict(base_metadata)
        metadata.update({
            'queue_delay_s': max(
                0.0, inference_started_at - segment['queued_at']),
            'vlm_inference_s': max(
                0.0, inference_finished_at - inference_started_at),
            'vlm_skipped': False,
            'last_vlm_at': inference_finished_at,
        })
        memory_id = self.store.add(
            caption, middle, frame_count=len(frames), observed_at=observed,
            metadata=metadata)
        with self.lock:
            self.last_memory_at = time.time()
            self.last_error = ''
        return 'inferred' if memory_id else None

    def add_manual(self, caption):
        pose, reason = self.navigation.memory_pose_snapshot()
        if pose is None:
            return False, reason, None
        try:
            memory_id = self.store.add(
                caption, pose, frame_count=0,
                metadata={'source': 'manual', 'images_persisted': False})
        except Exception as exc:
            return False, '记忆写入失败: %s' % str(exc)[:300], None
        with self.lock:
            self.last_memory_at = time.time()
            self.last_error = ''
        return True, '记忆已写入当前地图位姿', memory_id

    def query(self, request):
        if bool(request.get('execute', False)):
            return {
                'success': False,
                'message': '语义记忆接口不允许直接执行运动，请确认候选点后使用导航接口',
                'memories': [], 'candidate': None,
            }
        identity, reason = self.navigation.memory_map_snapshot()
        if identity is None:
            return {'success': False, 'message': reason,
                    'memories': [], 'candidate': None}
        try:
            limit = min(self.max_limit, max(1, int(
                request.get('limit', self.default_limit))))
            mode = str(request.get('mode', 'text')).strip().lower()
            if mode == 'text':
                text = str(request.get('text', '')).strip()
                if not text:
                    raise ValueError('请输入要查找的地点或物体')
                memories = self.store.search_text(
                    text, identity['map_signature'], limit)
            elif mode == 'position':
                x, y = float(request['x']), float(request['y'])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError('位置坐标无效')
                radius = request.get('radius')
                radius = None if radius in (None, '') else float(radius)
                if radius is not None and (not math.isfinite(radius) or radius < 0):
                    raise ValueError('位置半径无效')
                memories = self.store.search_position(
                    x, y, identity['map_signature'], limit, radius)
            elif mode == 'time':
                since = request.get('since')
                until = request.get('until')
                since = None if since in (None, '') else float(since)
                until = None if until in (None, '') else float(until)
                if ((since is not None and not math.isfinite(since)) or
                        (until is not None and not math.isfinite(until))):
                    raise ValueError('时间范围无效')
                memories = self.store.search_time(
                    identity['map_signature'], limit,
                    since, until)
            else:
                raise ValueError('查询 mode 只能是 text、position 或 time')
        except (KeyError, TypeError, ValueError) as exc:
            return {'success': False, 'message': str(exc),
                    'memories': [], 'candidate': None}
        except Exception as exc:
            return {
                'success': False,
                'message': '记忆检索后端失败: %s' % str(exc)[:300],
                'memories': [], 'candidate': None,
            }

        candidate = None
        candidate_reason = '当前地图没有匹配记忆'
        current_pose, pose_reason = self.navigation.memory_pose_snapshot()
        if memories and current_pose is not None:
            query_text = str(request.get('text', '')).strip()
            try:
                selected_id = (self.reasoner.select(query_text, memories)
                               if self.reasoner.ready else memories[0]['id'])
            except Exception:
                selected_id = memories[0]['id']
            target = next(
                (item for item in memories if item['id'] == selected_id), memories[0])
            try:
                ok, plan_message, preview = self.navigation.preview_plan(
                    current_pose['x'], current_pose['y'], target['x'], target['y'])
            except Exception as exc:
                ok, preview = False, None
                plan_message = '候选路径预演失败: %s' % str(exc)[:300]
            candidate_reason = plan_message
            if ok:
                candidate = {
                    'memory_id': target['id'],
                    'x': preview['goal']['x'],
                    'y': preview['goal']['y'],
                    'caption': target['caption'],
                    'preview': preview,
                    'requires_confirmation': True,
                }
        elif memories:
            candidate_reason = pose_reason
        return {
            'success': True,
            'message': ('找到 %d 条当前地图记忆；%s' %
                        (len(memories), candidate_reason)),
            'memories': memories,
            'candidate': candidate,
        }

    def status(self):
        identity, identity_reason = self.navigation.memory_map_snapshot()
        signature = identity['map_signature'] if identity else None
        vision_online = self.vision.online()
        try:
            memory_count = self.store.count(signature) if signature else 0
            total_memory_count = self.store.count()
            store_error = ''
        except Exception as exc:
            memory_count = 0
            total_memory_count = 0
            store_error = '数据库状态读取失败: %s' % str(exc)[:300]
        with self.lock:
            enabled = self.enabled
            frame_count = len(self.segment_frames)
            last_error = self.last_error
            started_at = self.started_at
            last_memory_at = self.last_memory_at
            thread_alive = self.thread is not None and self.thread.is_alive()
            inference_thread_alive = (
                self.inference_thread is not None and
                self.inference_thread.is_alive())
            pending = self.pending_segment is not None
            inference_active = self.inference_active
            last_inference_s = self.last_inference_s
            segments_completed = self.segments_completed
            segments_processed = self.segments_processed
            segments_dropped = self.segments_dropped
            single_frame_segments = self.single_frame_segments
            multi_frame_segments = self.multi_frame_segments
            keyframe_refreshes = self.keyframe_refreshes
            last_refresh_s = self.last_refresh_s
        return {
            'enabled': enabled,
            'capture_thread_alive': thread_alive,
            'ready': bool(
                self.vision.ready and vision_online and self.embedding.ready),
            'vlm': {'backend': self.vision.name, 'model': self.vision.model,
                    'ready': bool(self.vision.ready), 'online': vision_online},
            'embedding': {
                'backend': self.embedding.name, 'model': self.embedding.model,
                'ready': bool(self.embedding.ready),
            },
            'reasoner': {
                'backend': self.reasoner.name, 'model': self.reasoner.model,
                'ready': bool(self.reasoner.ready),
            },
            'database_path': self.store.path,
            'config_path': self.config_path,
            'map_ready': identity is not None,
            'map_reason': identity_reason,
            'memory_count': memory_count,
            'total_memory_count': total_memory_count,
            'deduplication': self.store.deduplication_status(),
            'segment_frame_count': frame_count,
            'pipeline': {
                'queue_policy': 'latest_only',
                'sample_interval_s': self.sample_interval,
                'segment_duration_s': self.segment_duration,
                'frames_per_segment': self.frames_per_segment,
                'adaptive_frames': self.adaptive_frames,
                'adaptive_frame_hamming': self.adaptive_frame_hamming,
                'keyframe_filter_enabled': self.keyframe_filter_enabled,
                'keyframe_filter_hamming': self.keyframe_filter_hamming,
                'keyframe_filter_max_vlm_age_s':
                    self.keyframe_filter_max_vlm_age,
                'pending_segments': 1 if pending else 0,
                'inference_active': inference_active,
                'inference_thread_alive': inference_thread_alive,
                'segments_completed': segments_completed,
                'segments_processed': segments_processed,
                'segments_dropped': segments_dropped,
                'single_frame_segments': single_frame_segments,
                'multi_frame_segments': multi_frame_segments,
                'keyframe_refreshes': keyframe_refreshes,
                'last_inference_s': round(last_inference_s, 3),
                'last_refresh_s': round(last_refresh_s, 3),
            },
            'started_at': round(started_at, 3) if started_at else None,
            'last_memory_at': round(last_memory_at, 3) if last_memory_at else None,
            'last_error': last_error or store_error or None,
            'images_persisted': False,
        }

    def shutdown(self):
        self.set_enabled(False)
        with self.condition:
            self.worker_stop_event.set()
            self.condition.notify_all()
            inference_thread = self.inference_thread
        if (inference_thread and
                inference_thread is not threading.current_thread()):
            inference_thread.join(timeout=2.0)
        try:
            self.store.close()
        except Exception:
            pass


class UnavailableRemembrService:
    """Fail-open for Web availability, fail-closed for semantic operations."""

    def __init__(self, error):
        self.error = str(error)[:500]

    def set_enabled(self, enabled):
        return False, '语义记忆服务不可用: %s' % self.error

    def add_manual(self, caption):
        return False, '语义记忆服务不可用: %s' % self.error, None

    def query(self, request):
        return {
            'success': False,
            'message': '语义记忆服务不可用: %s' % self.error,
            'memories': [], 'candidate': None,
        }

    def status(self):
        return {
            'enabled': False,
            'capture_thread_alive': False,
            'ready': False,
            'vlm': {'backend': 'unavailable', 'model': '', 'ready': False,
                    'online': False},
            'embedding': {'backend': 'unavailable', 'model': '', 'ready': False},
            'reasoner': {'backend': 'unavailable', 'model': '', 'ready': False},
            'deduplication': {'enabled': False},
            'map_ready': False,
            'memory_count': 0,
            'total_memory_count': 0,
            'pipeline': {
                'queue_policy': 'latest_only',
                'pending_segments': 0,
                'inference_active': False,
                'inference_thread_alive': False,
                'segments_completed': 0,
                'segments_processed': 0,
                'segments_dropped': 0,
                'single_frame_segments': 0,
                'multi_frame_segments': 0,
                'keyframe_refreshes': 0,
                'last_inference_s': 0.0,
                'last_refresh_s': 0.0,
            },
            'last_error': self.error,
            'images_persisted': False,
        }

    def shutdown(self):
        return None
