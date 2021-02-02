import pty
import os
import serial
import threading

from ros2sim.parsers import Parser
from ros2sim.parsers import special as s


class SerialSimulator:
  def __init__(self, parser: Parser, baudrate=9600):
    """Create a new Serial simulator

    Args:
      parser (Parser): Which parser to use
    """
    self.parser = parser

  def serve(self):
    self.master, slave = pty.openpty()
    s_name = os.ttyname(slave)

    thread = threading.Thread(target=self.listener)
    thread.start()

    print("""
    Realtime Simulation now active.
    Serving on {}
    """.format(s_name))

  def listener(self):
    while True:
      request = b''
      while not request.endswith(s.EOM.value):
        request += os.read(self.master, 1)

      response = self.parser.parse(request)
      os.write(self.master, response + s.EOM.value)