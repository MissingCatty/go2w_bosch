"""SQLite-backed, map-scoped semantic memory store."""

import json
import math
import os
import sqlite3
import struct
import threading
import time
import uuid


SCHEMA_VERSION = 1


def _pack_vector(vector):
    values = [float(value) for value in vector]
    return struct.pack('<%df' % len(values), *values)


def _unpack_vector(blob):
    if not blob or len(blob) % 4:
        return []
    return list(struct.unpack('<%df' % (len(blob) // 4), blob))


def _cosine_normalized(left, right):
    if len(left) != len(right) or not left:
        return None
    return sum(a * b for a, b in zip(left, right))


class MemoryStore:
    """A single WAL database avoids a heavyweight vector service on the robot."""

    def __init__(self, path, embedding_backend, max_scan=50000,
                 deduplication=None):
        self.path = os.path.abspath(os.path.expanduser(str(path)))
        self.embedding = embedding_backend
        self.max_scan = max(1, int(max_scan))
        deduplication = (deduplication
                         if isinstance(deduplication, dict) else {})
        self.dedup_enabled = bool(deduplication.get('enabled', True))
        self.dedup_position_radius = max(
            0.0, float(deduplication.get('position_radius_m', 0.35)))
        self.dedup_z_tolerance = max(
            0.0, float(deduplication.get('z_tolerance_m', 0.30)))
        yaw_degrees = max(
            0.0, min(180.0, float(
                deduplication.get('yaw_tolerance_deg', 20.0))))
        self.dedup_yaw_tolerance = math.radians(yaw_degrees)
        self.lock = threading.RLock()
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.connection = sqlite3.connect(
            self.path, timeout=20.0, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        with self.lock:
            self.connection.execute('PRAGMA journal_mode=WAL')
            self.connection.execute('PRAGMA synchronous=NORMAL')
            self.connection.executescript('''
                CREATE TABLE IF NOT EXISTS memories (
                    id TEXT PRIMARY KEY,
                    observed_at REAL NOT NULL,
                    inserted_at REAL NOT NULL,
                    map_signature TEXT NOT NULL,
                    map_source TEXT NOT NULL,
                    navigation_session_id TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL NOT NULL,
                    yaw REAL NOT NULL,
                    caption TEXT NOT NULL,
                    embedding_backend TEXT NOT NULL,
                    embedding_model TEXT NOT NULL,
                    embedding BLOB NOT NULL,
                    frame_count INTEGER NOT NULL,
                    metadata_json TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_memories_map_time
                    ON memories(map_signature, observed_at DESC);
                CREATE INDEX IF NOT EXISTS idx_memories_map_xy
                    ON memories(map_signature, x, y);
                CREATE TABLE IF NOT EXISTS metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );
            ''')
            self.connection.execute(
                'INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)',
                ('schema_version', str(SCHEMA_VERSION)))
            self.connection.commit()

    def close(self):
        with self.lock:
            self.connection.close()

    @staticmethod
    def _yaw_distance(left, right):
        """Return the shortest absolute angular distance in radians."""
        return abs((float(left) - float(right) + math.pi) %
                   (2.0 * math.pi) - math.pi)

    def _similar_ids_locked(self, map_signature, x, y, z, yaw):
        if not self.dedup_enabled:
            return []
        radius = self.dedup_position_radius
        rows = self.connection.execute('''
            SELECT id, x, y, z, yaw FROM memories
            WHERE map_signature = ?
              AND x BETWEEN ? AND ?
              AND y BETWEEN ? AND ?
        ''', (map_signature, x - radius, x + radius,
              y - radius, y + radius)).fetchall()
        return [row['id'] for row in rows
                if math.hypot(float(row['x']) - x,
                              float(row['y']) - y) <= radius
                and abs(float(row['z']) - z) <= self.dedup_z_tolerance
                and self._yaw_distance(row['yaw'], yaw) <=
                    self.dedup_yaw_tolerance]

    def deduplication_status(self):
        return {
            'enabled': self.dedup_enabled,
            'position_radius_m': self.dedup_position_radius,
            'z_tolerance_m': self.dedup_z_tolerance,
            'yaw_tolerance_deg': round(
                math.degrees(self.dedup_yaw_tolerance), 3),
        }

    def refresh_similar_if_unchanged(
            self, pose, fingerprint, max_hamming, max_vlm_age_s,
            frame_count=0, metadata=None, observed_at=None):
        """Refresh one recent similar memory without rerunning VLM/embedding.

        This is intentionally stricter than pose deduplication: a stored
        perceptual fingerprint must also be close, and a periodic VLM refresh
        is forced once max_vlm_age_s expires.
        """
        if (not self.dedup_enabled or not isinstance(fingerprint, int) or
                not 0 <= fingerprint < (1 << 64)):
            return None
        max_hamming = max(0, min(64, int(max_hamming)))
        max_vlm_age_s = max(0.0, float(max_vlm_age_s))
        map_signature = str(pose['map_signature'])
        x, y = float(pose['x']), float(pose['y'])
        z, yaw = float(pose.get('z', 0.0)), float(pose.get('yaw', 0.0))
        now = time.time()
        observed = float(observed_at if observed_at is not None else now)
        with self.lock:
            self.connection.execute('BEGIN IMMEDIATE')
            try:
                similar_ids = self._similar_ids_locked(
                    map_signature, x, y, z, yaw)
                if not similar_ids:
                    self.connection.rollback()
                    return None
                placeholders = ','.join('?' for _ in similar_ids)
                rows = self.connection.execute(
                    'SELECT * FROM memories WHERE map_signature = ? '
                    'AND id IN (%s)' % placeholders,
                    [map_signature] + similar_ids).fetchall()
                candidates = []
                for row in rows:
                    try:
                        stored_metadata = json.loads(row['metadata_json'])
                        stored_fingerprint = int(
                            stored_metadata['perceptual_fingerprint'])
                        if not 0 <= stored_fingerprint < (1 << 64):
                            continue
                        last_vlm_at = float(stored_metadata.get(
                            'last_vlm_at', row['inserted_at']))
                    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                        continue
                    distance = bin(fingerprint ^ stored_fingerprint).count('1')
                    vlm_age = max(0.0, now - last_vlm_at)
                    if (distance <= max_hamming and
                            vlm_age <= max_vlm_age_s):
                        candidates.append((
                            distance, -float(row['observed_at']),
                            str(row['id']), row,
                            stored_metadata, last_vlm_at))
                if not candidates:
                    self.connection.rollback()
                    return None
                _, _, _, selected, stored_metadata, last_vlm_at = min(candidates)
                other_ids = [item for item in similar_ids
                             if item != selected['id']]
                if other_ids:
                    other_placeholders = ','.join('?' for _ in other_ids)
                    self.connection.execute(
                        'DELETE FROM memories WHERE map_signature = ? '
                        'AND id IN (%s)' % other_placeholders,
                        [map_signature] + other_ids)
                refreshed_metadata = dict(stored_metadata)
                refreshed_metadata.update(dict(metadata or {}))
                refreshed_metadata.update({
                    'last_vlm_at': last_vlm_at,
                    'keyframe_refresh_count': int(stored_metadata.get(
                        'keyframe_refresh_count', 0)) + 1,
                    'replaced_similar_count': len(other_ids),
                })
                self.connection.execute('''
                    UPDATE memories SET
                        observed_at = ?, inserted_at = ?, map_source = ?,
                        navigation_session_id = ?, x = ?, y = ?, z = ?, yaw = ?,
                        frame_count = ?, metadata_json = ?
                    WHERE id = ? AND map_signature = ?
                ''', (
                    observed, now, str(pose.get('map_source', '')),
                    str(pose.get('navigation_session_id', '')),
                    x, y, z, yaw, int(frame_count),
                    json.dumps(refreshed_metadata, ensure_ascii=False,
                               separators=(',', ':')),
                    selected['id'], map_signature,
                ))
                self.connection.commit()
            except Exception:
                self.connection.rollback()
                raise
        return selected['id']

    def add(self, caption, pose, frame_count=0, metadata=None, observed_at=None):
        caption = str(caption).strip()
        if not caption:
            raise ValueError('记忆描述不能为空')
        vector = self.embedding.embed(caption)
        if not vector:
            raise ValueError('Embedding 结果为空')
        memory_id = uuid.uuid4().hex
        now = time.time()
        observed = float(observed_at if observed_at is not None else now)
        map_signature = str(pose['map_signature'])
        x, y = float(pose['x']), float(pose['y'])
        z, yaw = float(pose.get('z', 0.0)), float(pose.get('yaw', 0.0))
        with self.lock:
            self.connection.execute('BEGIN IMMEDIATE')
            try:
                similar_ids = self._similar_ids_locked(
                    map_signature, x, y, z, yaw)
                if similar_ids:
                    placeholders = ','.join('?' for _ in similar_ids)
                    self.connection.execute(
                        'DELETE FROM memories WHERE map_signature = ? '
                        'AND id IN (%s)' % placeholders,
                        [map_signature] + similar_ids)
                stored_metadata = dict(metadata or {})
                if similar_ids:
                    stored_metadata['replaced_similar_count'] = len(similar_ids)
                values = (
                    memory_id, observed, now, map_signature,
                    str(pose.get('map_source', '')),
                    str(pose.get('navigation_session_id', '')),
                    x, y, z, yaw, caption,
                    str(self.embedding.name), str(self.embedding.model),
                    sqlite3.Binary(_pack_vector(vector)), int(frame_count),
                    json.dumps(stored_metadata, ensure_ascii=False,
                               separators=(',', ':')),
                )
                self.connection.execute('''
                    INSERT INTO memories(
                        id, observed_at, inserted_at, map_signature, map_source,
                        navigation_session_id, x, y, z, yaw, caption,
                        embedding_backend, embedding_model, embedding, frame_count,
                        metadata_json)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', values)
                self.connection.commit()
            except Exception:
                self.connection.rollback()
                raise
        return memory_id

    @staticmethod
    def _public(row, score=None, distance=None):
        result = {
            'id': row['id'],
            'observed_at': round(float(row['observed_at']), 3),
            'x': round(float(row['x']), 3),
            'y': round(float(row['y']), 3),
            'z': round(float(row['z']), 3),
            'yaw': round(float(row['yaw']), 4),
            'caption': row['caption'],
            'frame_count': int(row['frame_count']),
        }
        if score is not None:
            result['score'] = round(float(score), 5)
        if distance is not None:
            result['distance'] = round(float(distance), 3)
        return result

    def search_text(self, text, map_signature, limit=5):
        query = self.embedding.embed(str(text))
        if not query:
            return []
        with self.lock:
            rows = self.connection.execute('''
                SELECT * FROM memories
                WHERE map_signature = ? AND embedding_backend = ?
                      AND embedding_model = ?
                ORDER BY observed_at DESC LIMIT ?
            ''', (str(map_signature), str(self.embedding.name),
                  str(self.embedding.model), self.max_scan)).fetchall()
        ranked = []
        for row in rows:
            score = _cosine_normalized(query, _unpack_vector(row['embedding']))
            if score is not None:
                ranked.append((score, float(row['observed_at']), row))
        ranked.sort(key=lambda item: (item[0], item[1]), reverse=True)
        return [self._public(row, score=score)
                for score, _, row in ranked[:max(1, int(limit))]]

    def search_position(self, x, y, map_signature, limit=5, radius=None):
        x, y = float(x), float(y)
        with self.lock:
            rows = self.connection.execute('''
                SELECT * FROM memories WHERE map_signature = ?
                ORDER BY ((x - ?) * (x - ?) + (y - ?) * (y - ?)) ASC
                LIMIT ?
            ''', (str(map_signature), x, x, y, y,
                  min(self.max_scan, max(1, int(limit)) * 4))).fetchall()
        results = []
        for row in rows:
            distance = math.hypot(float(row['x']) - x, float(row['y']) - y)
            if radius is None or distance <= float(radius):
                results.append(self._public(row, distance=distance))
            if len(results) >= max(1, int(limit)):
                break
        return results

    def search_time(self, map_signature, limit=5, since=None, until=None):
        clauses = ['map_signature = ?']
        values = [str(map_signature)]
        if since is not None:
            clauses.append('observed_at >= ?')
            values.append(float(since))
        if until is not None:
            clauses.append('observed_at <= ?')
            values.append(float(until))
        values.append(max(1, int(limit)))
        sql = ('SELECT * FROM memories WHERE ' + ' AND '.join(clauses) +
               ' ORDER BY observed_at DESC LIMIT ?')
        with self.lock:
            rows = self.connection.execute(sql, values).fetchall()
        return [self._public(row) for row in rows]

    def count(self, map_signature=None):
        with self.lock:
            if map_signature:
                row = self.connection.execute(
                    'SELECT COUNT(*) AS count FROM memories WHERE map_signature = ?',
                    (str(map_signature),)).fetchone()
            else:
                row = self.connection.execute(
                    'SELECT COUNT(*) AS count FROM memories').fetchone()
        return int(row['count'])
