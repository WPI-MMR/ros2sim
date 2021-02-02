from ros2sim import parsers

import json


class JsonParser(parsers.Parser):
  def __init__(self, env):
    super().__init__(env)

    # TODO: Talk to @andrew_103 about how exactly the parsing and deparsing
    # work for embedded systems. Need to evaluate if this state machine logic
    # can be abstracted abstracted out to the parent class.
    self._COMMAND_MAPPING = {
      parsers.special.RESET: self.reset,
      parsers.special.OBS_REQUEST: self.get_obs,
      parsers.special.ACTION: self.action,
    }

  def parse(self, input_str: str) -> str:
    for cmd, func in self._COMMAND_MAPPING.items():
      if input_str.startswith(cmd):
        resp = func(cmd)
        break

    resp = resp or parsers.special.OK
    return resp

  def reset(self):
    pass

  def get_obs(self):
    pass

  def action(self):
    pass