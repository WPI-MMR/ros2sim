from ros2sim.parsers import sim_executor, JointInformation, SerialReadState
from ros2sim.parsers import special as s

from parameterized import parameterized
from unittest import mock
import json

import unittest


class TestSimExecutor(unittest.TestCase):
  def setUp(self):
    self.env_stub = mock.MagicMock()
    self.env_stub.joint_ordering = ['FL_HFE', 'FL_KFE', 'FL_ANKLE', 'FR_HFE',
      'FR_KFE', 'FR_ANKLE', 'HL_HFE', 'HL_KFE', 'HL_ANKLE', 'HR_HFE', 'HR_KFE',
      'HR_ANKLE']
    self.sim_executor = sim_executor.SimExecutor(self.env_stub)

  def test_action(self):
    joint_values = {"left_hip":1, "left_knee":2, "right_hip":3, "right_knee":4,
      "left_shoulder":5, "left_elbow":6, "right_shoulder":7, "right_elbow":8}
    packet = JointInformation()
    packet.set_all_joint_values(**joint_values)
    ground_truth = [joint_values["left_shoulder"], joint_values["left_elbow"], 0,
      joint_values["right_shoulder"], joint_values["right_elbow"], 0,
      joint_values["left_hip"], joint_values["left_knee"], 0,
      joint_values["right_hip"], joint_values["right_knee"], 0]
    self.sim_executor.action(packet=packet)
    self.env_stub.step.assert_called_once_with(ground_truth)
    self.assertEqual(self.sim_executor.current_goal, ground_truth)

  @parameterized.expand([
    [[10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21], True],
    [[15, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21], False],
  ])
  def test_get_obs(self, current_goal, at_goal):
    values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      13, 14, 15, 16, 17, 18, 19, 20, 21]
    labels =  ['θx', 'θy', 'θz', 'vx', 'vy', 'vz',
      'wx', 'wy', 'wz', 'FL_HFE', 'FL_KFE', 'FL_ANKLE', 'FR_HFE', 'FR_KFE',
      'FR_ANKLE', 'HL_HFE', 'HL_KFE', 'HL_ANKLE', 'HR_HFE', 'HR_KFE', 'HR_ANKLE']
    self.env_stub.get_obs.return_value = (values, labels)
    self.sim_executor.current_goal = current_goal
    observation_packet = self.sim_executor.get_obs()

    for i, theta in enumerate(labels[:3]):
      self.assertEqual(observation_packet.get_theta_value(theta), values[i])

    for i, label in enumerate(labels[9:]):
      # Don't consider the dummy angle joint
      if label[-5:] != "ANKLE":
        self.assertEqual(
          observation_packet.get_joint_value_from_state(
            self.sim_executor.joint_ordering_to_read_state[label]
          ), values[i+9]
        )
    
    self.assertEqual(observation_packet.at_goal, at_goal)

  def test_reset(self):
    self.sim_executor.reset()
    self.env_stub.reset.assert_called_once()

    

if __name__ == '__main__':
  unittest.main()