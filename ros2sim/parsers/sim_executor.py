from typing import Text

import json

from ros2sim.parsers import SerialReadState, JointInformation

import numpy as np
import math

class SimExecutor():
  """Executes the necessary task based on the data recived from ROS
  """
  def __init__(self, env):
    self.env = env
    self.joint_ordering_to_read_state = {
      'FL_HFE': SerialReadState.READ_L_SHOULDER,
      'FL_KFE': SerialReadState.READ_L_ELBOW,
      'FR_HFE': SerialReadState.READ_R_SHOULDER,
      'FR_KFE': SerialReadState.READ_R_ELBOW,
      'HL_HFE': SerialReadState.READ_L_HIP,
      'HL_KFE': SerialReadState.READ_L_KNEE,
      'HR_HFE': SerialReadState.READ_R_HIP,
      'HR_KFE': SerialReadState.READ_R_KNEE
    }
    # List of observation labels we care about.
    self.obs_list = ['θx', 'θy', 'θz', 'FL_HFE', 'FL_KFE', 'FR_HFE',
      'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']
    
    # This should be the numpy array of joint values corresponding to the
    # joint ordering in self.env.joint_ordering
    self.current_goal = None
    # Acceptable tolerance in degrees. Can be changed to 0 if we want to only consider
    # perfect scenerios
    self.acceptable_tolerance = 2

  def reset(self):
    """Reset the environment."""
    self.env.reset()

  def get_obs(self) -> JointInformation:
    """Get the current observation of the robot.

    Returns:
      JointInformation: IMU and joint values stored in JointInformation class
    """
    # The first 9 elements in value are regarding orientation and
    # the next 12 are joint angle values. Labels and values have the same length
    # and each value corresponds to the label it has the same index with in
    # their respective lists
    # NOTE: Values are floating points but we only use ints for communication.
    # TODO: Modify get obs() to only return ints
    values, labels = self.env.get_obs()
    values = np.degrees(values)
    for i, deg in enumerate(values):
      if deg < 0:
        values[i] += 360

    ## This is important to preserve the exact values of joint values.
    # Converting frop degrees to radians and back to degrees can cause some floating point
    # differences to exist. This essentially gets rid of them.
    values = np.round(values)

    observation_packet = JointInformation()
    for i, label in enumerate(self.obs_list[0:3]):
      observation_packet.set_theta_value(label, int(values[labels.index(label)]))
    for i, label in enumerate(self.obs_list[3:]):
      observation_packet.set_joint_value_from_state(
        self.joint_ordering_to_read_state[label], int(values[labels.index(label)])
      )
    observation_packet.at_goal = np.allclose(values[9:], np.degrees(self.current_goal),
      rtol=0, atol=self.acceptable_tolerance)
    return observation_packet

  def action(self, packet: JointInformation):
    """Apply the action to the robot. The values provided are in degrees. This is converted to radians
    in the range -pi to pi to be used with gym solo.

    Args:
      packet (JointInformation): A joint information packet that contains the joint values to be executed
    """
    try:     
      action_deg = []
      for joint in self.env.joint_ordering:
        # We want to ignore all the ankle joints
        if joint[-5:] != "ANKLE":
          action_deg.append(
            packet.get_joint_value_from_state(
              self.joint_ordering_to_read_state[joint]
            )
          )
        else:
          action_deg.append(0)
      
      # If the number of joint values provided has a mismatch with the number of joints in teh solo8 model
      if len(action_deg) != len(self.env.joint_ordering):
        raise ValueError
      
      action_rad = np.radians(action_deg)

      # Making sure the range is between -pi to pi
      for i, rad in enumerate(action_rad):
        if rad > math.pi:
          action_rad[i] -= (2 * math.pi)

      self.current_goal = action_rad
      self.env.step(action_rad)
    except:
      raise ValueError('The action needs to have the same number of joint as '
                       'the robot')

