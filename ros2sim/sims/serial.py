from ros2sim.parsers import Parser


class SerialSimulator:
  def __init__(self, parser: Parser):
    """Create a new Serial simulator

    Args:
      parser (Parser): Which parser to use
    """
    self.parser = parser