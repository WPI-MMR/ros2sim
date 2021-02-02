from ros2sim.parsers import json_parser
from ros2sim.parsers import special as s


from parameterized import parameterized
from unittest import mock

import unittest


class TestJsonParser(unittest.TestCase):
  def setUp(self):
    self.env_stub = mock.MagicMock()
    self.parser = json_parser.JsonParser(self.env_stub)

  @parameterized.expand([
    ('reset_empty', 'reset', s.RESET.value, b''),
    ('reset_extra', 'reset', s.RESET.value + b'test', b'test'),
    ('action_empty', 'action', s.ACTION.value, b''),
    ('action_extra', 'action', s.ACTION.value + b'test', b'test'),
    ('get_obs_empty', 'get_obs', s.OBS_REQUEST.value, b''),
    ('get_obs_extra', 'get_obs', s.OBS_REQUEST.value + b'test', b'test'),
  ])
  def test_parsing(self, name, funcname, cmd, input_str):
    with mock.patch.object(json_parser.JsonParser, funcname,
                           return_value=None) as mock_method:
      parser = json_parser.JsonParser(self.env_stub)
      resp = parser.parse(cmd)

      mock_method.assert_called_once_with(input_str)
      self.assertEqual(resp, s.OK.value)

  @parameterized.expand([
    ('simple', b''),
    ('extra_stuff', b'extra_stuff'),
  ])
  def test_reset(self, name, cmd):
    self.parser.reset(cmd)
    self.env_stub.reset.assert_called_once()

if __name__ == '__main__':
  unittest.main()