import unittest
from parameterized import parameterized

from unittest import TestCase
from unittest.mock import Mock, MagicMock, patch

from ros2sim.sims import SerialSimulator
from ros2sim.parsers import SimExecutor, SerialReadState, JointInformation

from typing import List

import logging

# So we don't see any logs during tests. Change this if you want to debug
logging.basicConfig(level=logging.CRITICAL)

class TestArduinoMocker(TestCase):
  
  def convert_to_bytes(self, inp: List) -> List:
    """Converts an list of ints into list of bytes
    """
    byte_inp = []
    for i in inp:
      byte_inp.append(bytes([i]))
    return byte_inp
  
  @parameterized.expand([
    ["correct_read", [255, 255, 255, 255, 1, 0, 2, 0, 3, 0, 4,
      0, 5, 0, 6, 0, 7, 0, 8, 0, 219], SerialReadState.READ_L_HIP],
    ["incorrect_read", [255, 255, 255, 254, 1, 0, 2, 0, 3, 0,
      4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 219], SerialReadState.READ_PREAMBLE]
  ])
  @patch('os.read')
  def test_preamble(self, name, side_effect, read_state, mock_read):
    
    parser = SimExecutor(None)
    serialsim = SerialSimulator(parser)
    
    # Defining a dummy value that is used during os.read()
    # This value does not affect the test cases.
    serialsim.master = "dummy_val"      

    mock_read.side_effect = self.convert_to_bytes(side_effect)
    preamble_count = serialsim.PREAMBLE_LENGTH

    for i in range(preamble_count):
      serialsim.recv_data()
    self.assertEqual(serialsim.preamble_counter, preamble_count)
    self.assertEqual(serialsim.serial_read_state, read_state)

  @parameterized.expand([
    ["wrong_checksum", [255, 255, 255, 255, 1, 0, 2, 0, 3, 0, 4, 0,
      5, 0, 6, 0, 7, 0, 8, 0, 255], True],
    ["correct_checksum", [255, 255, 255, 255, 1, 0, 2, 0, 3, 0, 4,
      0, 5, 0, 6, 0, 7, 0, 8, 0, 219], False]
  ])
  @patch('os.read')
  def test_checksum(self, name, side_effect, expected_result, mock_read):

    with self.subTest('Wrong checksum'):
      parser = SimExecutor(None)
      serialsim = SerialSimulator(parser)
      
      # Defining a dummy value that is used during os.read()
      # This value does not affect the test cases.
      serialsim.master = "dummy_val"      

      mock_read.side_effect = self.convert_to_bytes(side_effect)

      for i in range(len(side_effect)):
        serialsim.recv_data()

      self.assertEqual(serialsim.serial_read_state, SerialReadState.INIT)
      ground_truth = JointInformation()
      ground_truth.set_all_joint_values(left_hip=1, left_knee=2, right_hip=3, right_knee=4,
        left_shoulder=5, left_elbow=6, right_shoulder=7, right_elbow=8)
      
      self.assertEqual(serialsim.temp_packet_data.checksum_error, expected_result)

  @parameterized.expand([
    ["joint_angles_less_than_256", [255, 255, 255, 255, 1, 0, 2, 0, 3, 0,
      4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 219], {"left_hip":1, "left_knee":2, "right_hip":3, "right_knee":4,
      "left_shoulder":5, "left_elbow":6, "right_shoulder":7, "right_elbow":8}],
    ["joint_angles_equal_to_256", [255, 255, 255, 255, 255, 1, 255, 1, 255,
      1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255], {"left_hip":256, "left_knee":256, "right_hip":256,
      "right_knee":256, "left_shoulder":256, "left_elbow":256, "right_shoulder":256, "right_elbow":256}],
    ["joint_angles_greater_than_256", [255, 255, 255, 255, 255, 2, 255, 3, 255, 4, 255, 5, 255, 6, 255,
      7, 255, 8, 255, 9, 219], {"left_hip":257, "left_knee":258, "right_hip":259, "right_knee":260,
      "left_shoulder":261, "left_elbow":262, "right_shoulder":263, "right_elbow":264}]
  ])
  @patch('os.read')
  def test_joint_angles(self, name, side_effect, joint_values, mock_read):

    parser = SimExecutor(None)
    serialsim = SerialSimulator(parser)
    
    # Defining a dummy value that is used during os.read()
    # This value does not affect the test cases.
    serialsim.master = "dummy_val"      

    mock_read.side_effect = self.convert_to_bytes(side_effect)

    for i in range(len(side_effect)):
      serialsim.recv_data()

    self.assertEqual(serialsim.serial_read_state, SerialReadState.INIT)
    ground_truth = JointInformation()
    ground_truth.set_all_joint_values(**joint_values)
    
    # Checking if all the joint angles match the ground truth
    for state in serialsim.list_of_states[2:10]:
      self.assertEqual(serialsim.validated_packed_data.get_joint_value_from_state(state),
        ground_truth.get_joint_value_from_state(state))

  @patch('os.read')
  def test_robot_state(self, mock_read):

    inp = [255, 255, 255,
      255, 1, 1, 253]
    parser = SimExecutor(None)
    serialsim = SerialSimulator(parser)
    
    # Defining a dummy value that is used during os.read()
    # This value does not affect the test cases.
    serialsim.master = "dummy_val"      

    mock_read.side_effect = self.convert_to_bytes(inp)

    for i in range(len(inp)):
      serialsim.recv_data()

    self.assertEqual(serialsim.serial_read_state, SerialReadState.INIT)
    ground_truth = JointInformation()
    ground_truth.set_all_joint_values(left_hip=1, left_knee=2, right_hip=3, right_knee=4,
      left_shoulder=5, left_elbow=6, right_shoulder=7, right_elbow=8)
    
    self.assertEqual(serialsim.temp_packet_data.data_request, True)

