from typing import Text

import json

class SimExecutor():
  """Executes the necessary task based on the data recived from ROS
  """
  def __init__(self, env):
    self.env = env

  def reset(self):
    """Reset the environment."""
    self.env.reset()

  def get_obs(self) -> Text:
    """Get the current observation of the robot.

    Returns:
      str: A JSON-encoded string of a dictionary of the registered observation
        and the respective values.
    """
    values, labels = self.env.get_obs()
    return json.dumps(dict(zip(labels, values)))

  def action(self, cmd):
    """Apply the action to the robot.

    Note that in this case, these values will usually be motor position values.

    Args:
      cmd ([str]): JSON encoded dictionary of motor values
    """
    print(self.env.joint_ordering)
    action_dict = json.loads(cmd)
    try:
      action = [action_dict[joint] for joint in self.env.joint_ordering]
      if len(action) != len(self.env.joint_ordering):
        raise ValueError

      self.env.step(action)
    except:
      raise ValueError('The action needs to have the same number of joint as '
                       'the robot')
