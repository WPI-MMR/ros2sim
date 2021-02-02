from typing import Text

from ros2sim import parsers
import json


class JsonParser(parsers.Parser):
  def __init__(self, env):
    super().__init__(env)

    # TODO: Talk to @andrew_103 about how exactly the parsing and deparsing
    # works for embedded systems. Need to evaluate if this state machine logic
    # can be abstracted out to the parent class.
    self._COMMAND_MAPPING = {
      parsers.special.RESET: self.reset,
      parsers.special.OBS_REQUEST: self.get_obs,
      parsers.special.ACTION: self.action,
    }

  def parse(self, input_str: str) -> str:
    for cmd, func in self._COMMAND_MAPPING.items():
      if input_str.startswith(cmd.value):
        resp = func(input_str[len(cmd.value):])
        break

    resp = resp or parsers.special.OK.value
    return resp

  def reset(self, _):
    """Reset the environment."""
    self.env.reset()

  def get_obs(self, _) -> Text:
    """Get the current observation of the robot.

    Returns:
      str: A JSON-encoded string of a dictionary of the registered observation
        and the respective values.
    """
    labels, values = self.env.get_obs()
    return json.dumps(dict(zip(labels, values)))

  def action(self, cmd):
    """Apply the action to the robot.

    Note that in this case, these values will usually be motor position values.

    Args:
      cmd ([str]): JSON encoded dictionary of motor values
    """
    action_dict = json.loads(cmd)
    action = [action_dict[joint] for joint in self.env.joint_ordering]
    
    if len(action) != len(self.env.joint_ordering):
      raise ValueError('The action needs to have the same number of joint as '
                       'the robot')

    self.env.step(action)
