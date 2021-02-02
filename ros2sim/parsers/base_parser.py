from abc import ABC, abstractmethod
from typing import Text


class Parser(ABC):
  """A generic parser class."""
  def __init__(self, env):
    self.env = env

  @abstractmethod
  def parse(input_str: Text) -> Text:
    """React to an input string passed from the master device.

    As most robot communications are in the master-slave design pattern, this
    function will most likely just be a state machine that reacts to the 
    master node.

    Args:
      input_str (Text): Encoded string from master to decode.

    Returns:
      Text: A response (if needed--else can return an empty string)
    """
    pass