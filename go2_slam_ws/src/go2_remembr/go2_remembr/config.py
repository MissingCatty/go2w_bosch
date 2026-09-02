"""Configuration loading with a stable default and one optional override."""

import copy
import json
import os


DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'default_config.json')


def _merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path=None):
    """Load defaults and recursively merge ``GO2_REMEMBR_CONFIG`` if set."""
    with open(DEFAULT_CONFIG_PATH, 'r', encoding='utf-8') as stream:
        config = json.load(stream)
    override_path = path or os.environ.get('GO2_REMEMBR_CONFIG', '').strip()
    if override_path:
        with open(override_path, 'r', encoding='utf-8') as stream:
            override = json.load(stream)
        if not isinstance(override, dict):
            raise ValueError('ReMEmbR 配置根节点必须是 JSON 对象')
        config = _merge(config, override)
    return config, override_path or DEFAULT_CONFIG_PATH
