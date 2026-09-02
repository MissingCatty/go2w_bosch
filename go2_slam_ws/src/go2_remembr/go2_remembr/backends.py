"""Small, dependency-free model adapters suitable for an Orin edge process."""

import base64
import hashlib
import json
import math
import os
import re
import threading
import time
import unicodedata
import urllib.parse
import urllib.request


def _post_json(url, payload, timeout, api_key_env=''):
    headers = {'Content-Type': 'application/json'}
    if api_key_env:
        value = os.environ.get(api_key_env, '').strip()
        if value:
            headers['Authorization'] = 'Bearer ' + value
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode('utf-8'), headers=headers,
        method='POST')
    with urllib.request.urlopen(request, timeout=float(timeout)) as response:
        return json.loads(response.read().decode('utf-8'))


def _message_text(value):
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        parts = []
        for item in value:
            if isinstance(item, dict) and isinstance(item.get('text'), str):
                parts.append(item['text'])
        return '\n'.join(parts)
    return ''


class DisabledVisionBackend:
    name = 'disabled'
    model = ''

    @property
    def ready(self):
        return False

    def caption(self, frames, prompt):
        raise RuntimeError('VLM 尚未配置')

    def online(self):
        return False


class HttpVisionBackend:
    """Generic local endpoint: {prompt, images:[base64 JPEG]} -> caption/text."""

    name = 'simple_http'

    def __init__(self, config):
        self.endpoint = str(config.get('endpoint', '')).strip()
        self.model = str(config.get('model', '')).strip()
        self.timeout = float(config.get('timeout_s', 90))
        self.api_key_env = str(config.get('api_key_env', '')).strip()
        self.max_tokens = max(32, int(config.get('max_tokens', 384)))
        self.temperature = float(config.get('temperature', 0.1))
        extra_body = config.get('extra_body', {})
        self.extra_body = dict(extra_body) if isinstance(extra_body, dict) else {}
        configured_health = str(config.get('health_endpoint', '')).strip()
        if configured_health:
            self.health_endpoint = configured_health
        elif self.endpoint:
            parts = urllib.parse.urlsplit(self.endpoint)
            self.health_endpoint = urllib.parse.urlunsplit(
                (parts.scheme, parts.netloc, '/health', '', ''))
        else:
            self.health_endpoint = ''
        self.health_lock = threading.Lock()
        self.health_checked_at = 0.0
        self.health_online = False

    @property
    def ready(self):
        return bool(self.endpoint)

    def online(self):
        now = time.time()
        with self.health_lock:
            if now - self.health_checked_at < 2.0:
                return self.health_online
        online = False
        if self.health_endpoint:
            try:
                request = urllib.request.Request(
                    self.health_endpoint, method='GET')
                with urllib.request.urlopen(request, timeout=0.5) as response:
                    online = 200 <= int(response.status) < 300
            except Exception:
                online = False
        with self.health_lock:
            self.health_checked_at = time.time()
            self.health_online = online
        return online

    def caption(self, frames, prompt):
        payload = {
            'model': self.model,
            'prompt': prompt,
            'images': [base64.b64encode(frame).decode('ascii') for frame in frames],
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }
        payload.update(self.extra_body)
        result = _post_json(
            self.endpoint, payload, self.timeout, self.api_key_env)
        caption = result.get('caption', result.get('text', ''))
        if not isinstance(caption, str) or not caption.strip():
            raise ValueError('VLM 响应缺少 caption/text')
        return caption.strip()


class OpenAICompatibleVisionBackend(HttpVisionBackend):
    """Vision adapter for a local OpenAI-compatible chat-completions server."""

    name = 'openai_compatible'

    @property
    def ready(self):
        return bool(self.endpoint and self.model)

    def caption(self, frames, prompt):
        content = [{'type': 'text', 'text': prompt}]
        content.extend({
            'type': 'image_url',
            'image_url': {
                'url': 'data:image/jpeg;base64,' +
                       base64.b64encode(frame).decode('ascii'),
            },
        } for frame in frames)
        payload = {
            'model': self.model,
            'messages': [{'role': 'user', 'content': content}],
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }
        payload.update(self.extra_body)
        result = _post_json(
            self.endpoint, payload, self.timeout, self.api_key_env)
        try:
            caption = _message_text(result['choices'][0]['message']['content'])
        except (KeyError, IndexError, TypeError):
            caption = ''
        if not caption.strip():
            raise ValueError('VLM 响应缺少 choices[0].message.content')
        return caption.strip()


class HashEmbeddingBackend:
    """Stable signed feature hashing over Unicode character/word n-grams."""

    name = 'hash'

    def __init__(self, config):
        self.dimensions = max(32, min(4096, int(config.get('dimensions', 256))))
        self.model = str(config.get('model', 'hash-char-v1')).strip() or 'hash-char-v1'

    @property
    def ready(self):
        return True

    @staticmethod
    def _tokens(text):
        normalized = unicodedata.normalize('NFKC', str(text)).lower()
        compact = ''.join(character for character in normalized
                          if not character.isspace())
        tokens = re.findall(r'[\w]+', normalized, flags=re.UNICODE)
        tokens.extend(compact[index:index + width]
                      for width in (1, 2, 3)
                      for index in range(max(0, len(compact) - width + 1)))
        return tokens

    def embed(self, text):
        vector = [0.0] * self.dimensions
        for token in self._tokens(text):
            digest = hashlib.blake2b(token.encode('utf-8'), digest_size=8).digest()
            value = int.from_bytes(digest, byteorder='little', signed=False)
            index = value % self.dimensions
            vector[index] += -1.0 if value & (1 << 63) else 1.0
        norm = math.sqrt(sum(value * value for value in vector))
        if norm:
            vector = [value / norm for value in vector]
        return vector


class OpenAICompatibleEmbeddingBackend:
    name = 'openai_compatible'

    def __init__(self, config):
        self.endpoint = str(config.get('endpoint', '')).strip()
        self.model = str(config.get('model', '')).strip()
        self.timeout = float(config.get('timeout_s', 20))
        self.api_key_env = str(config.get('api_key_env', '')).strip()

    @property
    def ready(self):
        return bool(self.endpoint and self.model)

    def embed(self, text):
        result = _post_json(self.endpoint, {
            'model': self.model, 'input': str(text),
        }, self.timeout, self.api_key_env)
        try:
            vector = [float(value) for value in result['data'][0]['embedding']]
        except (KeyError, IndexError, TypeError, ValueError):
            raise ValueError('Embedding 响应缺少 data[0].embedding')
        norm = math.sqrt(sum(value * value for value in vector))
        return [value / norm for value in vector] if norm else vector


class RetrievalOnlyReasoner:
    """Deterministic fallback: the highest-scoring retrieved memory wins."""

    name = 'retrieval_only'
    model = ''

    @property
    def ready(self):
        return True

    def select(self, query, memories):
        return memories[0]['id'] if memories else None


class OpenAICompatibleReasoner:
    """OpenAI-compatible selector constrained to retrieved memory IDs.

    Network/authentication failures deliberately fall back to deterministic
    retrieval so a remote reasoner can never make semantic lookup unavailable.
    """

    name = 'openai_compatible'

    def __init__(self, config):
        self.endpoint = str(config.get('endpoint', '')).strip()
        self.model = str(config.get('model', '')).strip()
        self.timeout = float(config.get('timeout_s', 30))
        self.api_key_env = str(config.get('api_key_env', '')).strip()
        self.max_tokens = max(16, int(config.get('max_tokens', 64)))
        extra_body = config.get('extra_body', {})
        self.extra_body = dict(extra_body) if isinstance(extra_body, dict) else {}

    @property
    def ready(self):
        credential_ready = (not self.api_key_env or
                            bool(os.environ.get(self.api_key_env, '').strip()))
        return bool(self.endpoint and self.model and credential_ready)

    def select(self, query, memories):
        if not memories:
            return None
        candidates = [{key: item[key] for key in
                       ('id', 'caption', 'x', 'y', 'observed_at')}
                      for item in memories]
        payload = {
            'model': self.model,
            'messages': [{
                'role': 'user',
                'content': (
                    '从候选记忆中选择最符合查询的一条。只能返回 JSON '
                    '{"memory_id":"候选id"}，无法判断则返回 '
                    '{"memory_id":null}。\n查询：%s\n候选：%s' % (
                        str(query), json.dumps(candidates, ensure_ascii=False))),
            }],
            'temperature': 0.0,
            'max_tokens': self.max_tokens,
        }
        payload.update(self.extra_body)
        try:
            result = _post_json(
                self.endpoint, payload, self.timeout, self.api_key_env)
        except Exception:
            return memories[0]['id']
        try:
            content = _message_text(result['choices'][0]['message']['content']).strip()
            if content.startswith('```'):
                content = content.strip('`').strip()
                if content.startswith('json'):
                    content = content[4:].strip()
            selected = json.loads(content).get('memory_id')
        except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError):
            return memories[0]['id']
        allowed = {item['id'] for item in memories}
        return selected if selected in allowed else memories[0]['id']


def make_vision_backend(config):
    backend = str(config.get('backend', 'disabled')).strip().lower()
    if backend == 'disabled':
        return DisabledVisionBackend()
    if backend == 'simple_http':
        return HttpVisionBackend(config)
    if backend == 'openai_compatible':
        return OpenAICompatibleVisionBackend(config)
    raise ValueError('不支持的 VLM backend: %s' % backend)


def make_embedding_backend(config):
    backend = str(config.get('backend', 'hash')).strip().lower()
    if backend == 'hash':
        return HashEmbeddingBackend(config)
    if backend == 'openai_compatible':
        return OpenAICompatibleEmbeddingBackend(config)
    raise ValueError('不支持的 embedding backend: %s' % backend)


def make_reasoner(config):
    backend = str(config.get('backend', 'retrieval_only')).strip().lower()
    if backend == 'retrieval_only':
        return RetrievalOnlyReasoner()
    if backend == 'openai_compatible':
        return OpenAICompatibleReasoner(config)
    raise ValueError('不支持的 reasoner backend: %s' % backend)
