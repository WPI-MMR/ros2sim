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
    
    # This should be the list of joint values corresponding to the
    # joint ordering in self.env.joint_ordering
    self.current_goal = None
    # Acceptable tolerance in degrees.
    self.acceptable_tolerance = 2

  def reset(self):
    """Reset the environment."""
    self.env.reset()

  def get_obs(self) -> JointInformation:
    """Get the current observation of the robot.

    Returns:
      str: A JSON-encoded string of a dictionary of the registered observation
        and the respective values.
    """
    # The first 9 elements in value are regarding orientation and
    # the next 12 are joint angle values. Labels and values have the same length
    # and each value corresponds to the label it has the same index with in
    # their respective lists
    # NOTE: Values are floating points but we only use ints for communication.
    # TODO: Modify get obs() to only return ints
    values, labels = self.env.get_obs()
    values = np.degrees(values)
    print(values)
    observation_packet = JointInformation()
    for i, label in enumerate(self.obs_list[0:3]):
      observation_packet.set_theta_value(label, int(values[labels.index(label)]))
    for i, label in enumerate(self.obs_list[3:]):
      observation_packet.set_joint_value_from_state(
        self.joint_ordering_to_read_state[label], int(values[labels.index(label)])
      )
    observation_packet.at_goal = np.allclose(values[9:], self.current_goal,
      rtol=0, atol=self.acceptable_tolerance)
    print(observation_packet)
    return observation_packet

  def action(self, packet: JointInformation):
    """Apply the action to the robot.

    Note that in this case, these values will usually be motor position values.

    Args:
      cmd ([str]): JSON encoded dictionary of motor values
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
      if len(action_deg) != len(self.env.joint_ordering):
        raise ValueError
      action_rad = np.radians(action_deg)
      self.current_goal = action_rad
      print(action_rad)
      self.env.step(action_rad)
    except:
      raise ValueError('The action needs to have the same number of joint as '
                       'the robot')

