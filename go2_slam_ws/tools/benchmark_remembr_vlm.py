#!/usr/bin/env python3
import argparse
import base64
import json
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', nargs='+')
    parser.add_argument('--config', default='maps/remembr/config.json')
    parser.add_argument('--label', default='benchmark')
    parser.add_argument('--temperature', type=float, default=0.0)
    args = parser.parse_args()

    with open(args.config, encoding='utf-8') as handle:
        config = json.load(handle)
    vision = config['vlm']
    content = [{'type': 'text', 'text': config['capture']['prompt']}]
    for path in args.images:
        with open(path, 'rb') as handle:
            encoded = base64.b64encode(handle.read()).decode('ascii')
        content.append({
            'type': 'image_url',
            'image_url': {'url': 'data:image/jpeg;base64,' + encoded},
        })
    payload = {
        'model': vision['model'],
        'messages': [{'role': 'user', 'content': content}],
        'max_tokens': vision['max_tokens'],
        'temperature': args.temperature,
    }
    payload.update(vision.get('extra_body', {}))
    request = urllib.request.Request(
        vision['endpoint'], data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'}, method='POST')
    started = time.perf_counter()
    with urllib.request.urlopen(request, timeout=vision['timeout_s']) as response:
        result = json.load(response)
    elapsed = time.perf_counter() - started
    choice = result['choices'][0]
    print(json.dumps({
        'label': args.label,
        'wall_s': round(elapsed, 3),
        'finish_reason': choice.get('finish_reason'),
        'text': choice['message']['content'],
        'usage': result.get('usage', {}),
        'timings': result.get('timings', {}),
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
