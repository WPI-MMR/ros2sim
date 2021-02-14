from typing import Text

import json

from ros2sim.parsers import SerialReadState, JointInformation


class SimExecutor():
  """Executes the necessary task based on the data recived from ROS
  """
  def __init__(self, env):
    self.env = env
    self.joint_ordering_to_angle_dict = {
      'FL_HFE': SerialReadState.READ_L_SHOULDER,
      'FL_KFE': SerialReadState.READ_L_ELBOW,
      'FR_HFE': SerialReadState.READ_R_SHOULDER,
      'FR_KFE': SerialReadState.READ_R_ELBOW,
      'HL_HFE': SerialReadState.READ_L_HIP,
      'HL_KFE': SerialReadState.READ_L_KNEE,
      'HR_HFE': SerialReadState.READ_R_HIP,
      'HR_KFE': SerialReadState.READ_R_KNEE
    }
    self.obs_list = ['θx', 'θy', 'θz', 'FL_HFE', 'FL_KFE', 'FR_HFE',
      'FR_KFE', 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']

  def reset(self):
    """Reset the environment."""
    self.env.reset()

  def get_obs(self, validated_packed_data: JointInformation) -> JointInformation:
    """Get the current observation of the robot.

    Returns:
      str: A JSON-encoded string of a dictionary of the registered observation
        and the respective values.
    """
    values, labels = self.env.get_obs()
    for i, label in enumerate(self.obs_list[0:3]):
      validated_packed_data.set_theta_value(label, values[labels.index(label)])
    for i, label in enumerate(self.obs_list[3:]):
      validated_packed_data.set_joint_value_from_state(
        self.joint_ordering_to_angle_dict[label], values[labels.index(label)]
      )
    return validated_packed_data

  def action(self, packet: JointInformation):
    """Apply the action to the robot.

    Note that in this case, these values will usually be motor position values.

    Args:
      cmd ([str]): JSON encoded dictionary of motor values
    """
    try:
      action = [packet.get_joint_value_from_state(self.joint_ordering_to_angle_dict[joint])
        for joint in self.env.joint_ordering]
      if len(action) != len(self.env.joint_ordering):
        raise ValueError

      self.env.step(action)
    except:
      raise ValueError('The action needs to have the same number of joint as '
                       'the robot')
