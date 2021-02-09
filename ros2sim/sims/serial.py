import pty
import os
import serial
import threading

from ros2sim.parsers import Parser
from ros2sim.parsers import special as s
from ros2sim.parsers import SerialReadState
from ros2sim.parsers import JointInformation


class SerialSimulator:
  def __init__(self, parser: Parser, baudrate=9600):
    """Create a new Serial simulator

    Args:
      parser (Parser): Which parser to use
    """
    self.parser = parser
    self.PREAMBLE_LENGTH = 4
    self.DATA_BYTE_LENGTH = 2
    self.temp_packet_data = JointInformation()
    self.validated_packed_data = JointInformation()
    self.serial_read_state = SerialReadState.INIT
    self.serial_port_name = None

  def serve(self):
    self.master, slave = pty.openpty()
    s_name = os.ttyname(slave)
    self.slave = slave

    thread = threading.Thread(target=self.listener)
    thread.start()

    print("""
    Realtime Simulation now active.
    Serving on {}
    """.format(s_name))
    thread.join()

  def listener(self):
    """Listens to data being sent over the serial port.
    Depending on the data, it takes the appropriate action and responds back.
    """
    while True:
      self.recv_data()
    # while True:
    #   request = b''
    #   while not request.endswith(s.EOM.value):
    #     request += os.read(self.master, 1)

    #   response = self.parser.parse(request)
    #   os.write(self.master, response + s.EOM.value)

  def recv_data(self):
    """Checks to see if there is any data over the serial port to read. If there is,
    this method reads it and fills up the corresponding data structures. 
    """
    
    # Counter to check for the number of ints in preamble
    preamble_counter = self.PREAMBLE_LENGTH
    # Counter to check the number of for each joint angle
    data_byte_counter = self.DATA_BYTE_LENGTH
    # Running checksum used to determine the validity of the incoming packet.
    # We don't expect the mocking serial to have issues with the packed validity
    # but this is being implemented to keep with consistency of the Arduino code
    calculated_checksum = 0xFF
    # Data that is going to be read of serial. This is going to block here untill it reads.
    # Arduino has a non blocking version of this same code.
    # TODO: Evaluate if it can be made non blocking
    received_data = os.read(self.master, 1)

    if self.serial_read_state == SerialReadState.INIT:
      # Initializing checksum
      # TODO: Duplicated from the start of the method. Maybe remove from ther 
      calculated_checksum = 0x00
      preamble_counter = self.PREAMBLE_LENGTH
      # Reset the temp packet to store values in. We decide after reading checksum if the data
      # received is valid
      self.temp_packet_data.set_to_default()



    




    