from ros2sim.parsers import sim_executor, JointInformation, SerialReadState

from parameterized import parameterized
from unittest import mock

import json
import math
import numpy as np

import unittest


class TestSimExecutor(unittest.TestCase):
  def setUp(self):
    self.env_stub = mock.MagicMock()
    self.env_stub.joint_ordering = ['FL_HFE', 'FL_KFE', 'FL_ANKLE', 'FR_HFE',
      'FR_KFE', 'FR_ANKLE', 'HL_HFE', 'HL_KFE', 'HL_ANKLE', 'HR_HFE', 'HR_KFE',
      'HR_ANKLE']
    self.sim_executor = sim_executor.SimExecutor(self.env_stub)

  @parameterized.expand([
    # Joint values are given in degrees
    [{"left_hip":1, "left_knee":2, "right_hip":3, "right_knee":4, "left_shoulder":5,
      "left_elbow":6, "right_shoulder":7, "right_elbow":8}, [5,6,0,7,8,0,1,2,0,3,4,0]],
    [{"left_hip":256, "left_knee":256, "right_hip":256, "right_knee":256,
      "left_shoulder":256, "left_elbow":256, "right_shoulder":256, "right_elbow":256},
      [-104, -104, 0, -104, -104, 0, -104, -104, 0, -104, -104, 0]]
  ])
  def test_action(self, joint_values, ground_truth_deg):
    packet = JointInformation()
    packet.set_all_joint_values(**joint_values)
    ground_truth_deg = [joint_values["left_shoulder"], joint_values["left_elbow"], 0,
      joint_values["right_shoulder"], joint_values["right_elbow"], 0,
      joint_values["left_hip"], joint_values["left_knee"], 0,
      joint_values["right_hip"], joint_values["right_knee"], 0]
    ground_truth_rad = np.radians(ground_truth_deg)
    for i, rad in enumerate(ground_truth_rad):
      if rad > math.pi:
        ground_truth_rad[i] -= (2 * math.pi)
    #print(self.env_stub.step.call_args[0][0])
    self.sim_executor.action(packet=packet)
    self.assertTrue((ground_truth_rad == self.env_stub.step.call_args[0][0]).all())
    self.assertTrue((self.sim_executor.current_goal == ground_truth_rad).all())

  @parameterized.expand([
    [np.radians([10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]), True, 
    [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21],
    [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]],
    
    [np.radians([15, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]), False,
    [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21],
    [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]],
    
    [np.radians([270, 270, 270, 270, 270, 270, 270, 270, 270, 270, 270, 270]), True,
    [1, 2, 3, 4, 5, 6, 7, 8, 9, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90],
    [270, 270, 270, 270, 270, 270, 270, 270, 270, 270, 270, 270]]
  ])
  def test_get_obs(self, current_goal, at_goal, values_deg, ground_values):   
    values_rad = np.radians(values_deg)
    labels =  ['θx', 'θy', 'θz', 'vx', 'vy', 'vz',
      'wx', 'wy', 'wz', 'FL_HFE', 'FL_KFE', 'FL_ANKLE', 'FR_HFE', 'FR_KFE',
      'FR_ANKLE', 'HL_HFE', 'HL_KFE', 'HL_ANKLE', 'HR_HFE', 'HR_KFE', 'HR_ANKLE']
    self.env_stub.get_obs.return_value = (values_rad, labels)
    self.sim_executor.current_goal = current_goal
    observation_packet = self.sim_executor.get_obs()

    for i, theta in enumerate(labels[:3]):
      self.assertEqual(observation_packet.get_theta_value(theta), values_deg[i])

    for i, label in enumerate(labels[9:]):
      # Don't consider the dummy angle joint
      if label[-5:] != "ANKLE":
        self.assertEqual(
          observation_packet.get_joint_value_from_state(
            self.sim_executor.joint_ordering_to_read_state[label]
          ), ground_values[i]
        )
    
    self.assertEqual(observation_packet.at_goal, at_goal)

  def test_reset(self):
    self.sim_executor.reset()
    self.env_stub.reset.assert_called_once()

    

if __name__ == '__main__':
  unittest.main()