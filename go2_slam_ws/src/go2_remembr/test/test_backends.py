import unittest
from unittest import mock

from go2_remembr.backends import (
    OpenAICompatibleReasoner,
    OpenAICompatibleVisionBackend,
)


class VisionBackendTest(unittest.TestCase):
    def test_qwen_multimodal_request_keeps_runtime_controls(self):
        backend = OpenAICompatibleVisionBackend({
            'endpoint': 'http://127.0.0.1:8000/v1/chat/completions',
            'model': 'qwen3.5-2b-int8',
            'max_tokens': 256,
            'temperature': 0.2,
            'extra_body': {
                'top_k': 20,
                'chat_template_kwargs': {'enable_thinking': False},
            },
        })
        response = {
            'choices': [{'message': {'content': '  红色门旁有灭火器  '}}],
        }

        with mock.patch(
                'go2_remembr.backends._post_json', return_value=response) as post:
            caption = backend.caption([b'jpeg-a', b'jpeg-b'], '描述地点')

        self.assertEqual(caption, '红色门旁有灭火器')
        payload = post.call_args.args[1]
        self.assertEqual(payload['model'], 'qwen3.5-2b-int8')
        self.assertEqual(payload['max_tokens'], 256)
        self.assertEqual(payload['top_k'], 20)
        self.assertFalse(
            payload['chat_template_kwargs']['enable_thinking'])
        content = payload['messages'][0]['content']
        self.assertEqual(content[0], {'type': 'text', 'text': '描述地点'})
        self.assertEqual(len(content), 3)
        self.assertTrue(content[1]['image_url']['url'].startswith(
            'data:image/jpeg;base64,'))


class ReasonerBackendTest(unittest.TestCase):
    def _backend(self):
        return OpenAICompatibleReasoner({
            'endpoint': 'https://api.deepseek.com/chat/completions',
            'model': 'deepseek-v4-flash',
            'api_key_env': 'DEEPSEEK_API_KEY',
            'max_tokens': 64,
            'extra_body': {
                'thinking': {'type': 'disabled'},
                'response_format': {'type': 'json_object'},
            },
        })

    def test_deepseek_selector_uses_json_and_only_accepts_candidate_id(self):
        backend = self._backend()
        memories = [
            {'id': 'm1', 'caption': '红色门', 'x': 1.0, 'y': 2.0,
             'observed_at': 10.0},
            {'id': 'm2', 'caption': '灭火器', 'x': 3.0, 'y': 4.0,
             'observed_at': 20.0},
        ]
        response = {'choices': [{'message': {
            'content': '{"memory_id":"m2"}',
        }}]}
        with mock.patch.dict('os.environ', {'DEEPSEEK_API_KEY': 'test-key'}), \
                mock.patch('go2_remembr.backends._post_json',
                           return_value=response) as post:
            self.assertTrue(backend.ready)
            selected = backend.select('哪里有灭火器', memories)

        self.assertEqual(selected, 'm2')
        payload = post.call_args.args[1]
        self.assertEqual(payload['model'], 'deepseek-v4-flash')
        self.assertEqual(payload['max_tokens'], 64)
        self.assertEqual(payload['thinking'], {'type': 'disabled'})
        self.assertEqual(payload['response_format'], {'type': 'json_object'})

    def test_reasoner_is_not_ready_without_key_and_falls_back_on_network_error(self):
        backend = self._backend()
        memories = [
            {'id': 'm1', 'caption': '走廊', 'x': 1.0, 'y': 2.0,
             'observed_at': 10.0},
        ]
        with mock.patch.dict('os.environ', {}, clear=True):
            self.assertFalse(backend.ready)
        with mock.patch('go2_remembr.backends._post_json',
                        side_effect=OSError('断网')):
            self.assertEqual(backend.select('走廊', memories), 'm1')


if __name__ == '__main__':
    unittest.main()
