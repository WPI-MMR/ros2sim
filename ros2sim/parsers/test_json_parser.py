from ros2sim.parsers import json_parser
from ros2sim.parsers import special as s

from parameterized import parameterized
from unittest import mock
import json

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

  @parameterized.expand([
    ('simple', b''),
    ('extra_stuff', b'extra_stuff'),
  ])
  def test_get_obs(self, name, cmd):
    dummy_obs = ['lbl1', 'lbl2', 'lbl3'], [1., 2., 3.] 
    self.env_stub.get_obs.return_value = dummy_obs

    encoded_obs = {
      'lbl1': 1.,
      'lbl2': 2.,
      'lbl3': 3.,
    }
    encoded_str = json.dumps(encoded_obs)

    resp = self.parser.get_obs(cmd)

    self.env_stub.get_obs.assert_called_once()
    self.assertEqual(resp, encoded_str)

  @parameterized.expand([
    ('simple', {'j1': 1., 'j2': 2., 'j3': 3.,}, [1., 2., 3]), 
    ('extra', {'j1': 1., 'j2': 2., 'j3': 3., 'j4': 4.,}, [1., 2., 3]), 
  ])
  def test_action(self, name, sent_actions, expected_input):
    self.env_stub.joint_ordering = ['j1', 'j2', 'j3']
    self.parser.action(json.dumps(sent_actions))
    self.env_stub.step.assert_called_once_with(expected_input)

  def test_too_few_actions(self):
    self.env_stub.joint_ordering = ['j1', 'j2', 'j3']
    action = {'j1': 1., 'j2': 2.,}
    with self.assertRaises(ValueError):
      self.parser.action(json.dumps(action))
    

if __name__ == '__main__':
  unittest.main()