import pty
import os
import serial
import threading
import copy

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
    self.list_of_states = [state for state in SerialReadState]

  def serve(self):
    self.master, slave = pty.openpty()
    s_name = os.ttyname(slave)

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
      if self.validated_packed_data.packet_available:
        print(self.validated_packed_data)
        self.validated_packed_data.packet_available = False
        if validated_packed_data.request:
          print("Requesting data")
        else:
          print("Update Joint Angles")
          print(self.validated_packed_data)

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
    calculated_checksum = 255
    # Data that is going to be read of serial. This data is read as BYTES 
    # This is going to block here untill it reads.
    # Arduino has a non blocking version of this same code.
    # TODO: Evaluate if it can be made non blocking
    received_data = os.read(self.master, 1)
    # Converting the data from bytes to int
    received_data = int(received_data.hex(), 16)

    # TODO: Andrew what is the range of data being transferred.


    if self.serial_read_state == SerialReadState.INIT:
      # Initializing checksum
      # TODO: Duplicated from the start of the method. Maybe remove from there
      calculated_checksum = 0
      preamble_counter = self.PREAMBLE_LENGTH
      # Reset the temp packet to store values in. We decide after reading checksum if the data
      # received is valid
      self.temp_packet_data.set_to_default()
      # Changing serial read state to read preamble
      self.serial_read_state = SerialReadState.READ_PREAMBLE

    elif self.serial_read_state == SerialReadState.READ_PREAMBLE:
      if received_data == 255:
        preamble_counter -= 1
        if preamble_counter == 0:
          preamble_counter = self.PREAMBLE_LENGTH
          self.serial_read_state = SerialReadState.READ_L_HIP
      else:
        # TODO: Ask Andrew why do we even need this. If it is corrupt,
        # shouldn't we discard it?
        preamble_counter = self.PREAMBLE_LENGTH

    elif self.serial_read_state == SerialReadState.READ_L_HIP:
      # TODO: Ask Andrew about this condition. Also why couldn't we add a extra byte in the protocol.
      # The request is to get back the robot state
      if data_byte_counter == 1 and temp_packet_data.left_hip == received_data:
        self.temp_packet_data.data_request = True
        self.temp_packet_data.left_hip += received_data
        calculated_checksum += received_data
        data_byte_counter = self.DATA_BYTE_LENGTH
        self.serial_read_state = SerialReadState.READ_CHECKSUM
      else:
        self.temp_packet_data.left_hip += received_data
        calculated_checksum += received_data
        data_byte_counter -= 1
        if data_byte_counter == 0:
          data_byte_counter = self.DATA_BYTE_LENGTH
          self.serial_read_state = SerialReadState.READ_L_KNEE
    
    # Reading all the joint states except READ_L_HIP
    elif self.serial_read_state in self.list_of_states[3:10]:

      val = self.temp_packet_data.get_joint_value_from_state(self.serial_read_state)
      val += received_data
      self.temp_packet_data.set_joint_value_from_state(self.serial_read_state, val)

      calculated_checksum += received_data
      data_byte_counter -= 1
      if data_byte_counter == 0:
        data_byte_counter=self.DATA_BYTE_LENGTH
        self.serial_read_state = self.list_of_states[self.list_of_states.index(self.serial_read_state) + 1]

    elif self.serial_read_state == SerialReadState.READ_CHECKSUM:
      self.temp_packet_data.checksum = received_data

      if self.temp_packet_data == 255 - (calculated_checksum % 256):
        # This is a good packet

        # TODO: I don't think we need this here if we go to INIT next
        calculated_checksum = 0
        preamble_counter = self.PREAMBLE_LENGTH

        self.temp_packet_data.checksum_error = False
        self.temp_packet_data.packet_available = True

        self.validated_packed_data = copy.deepcopy(self.temp_packet_data)
      else:
        self.temp_packet_data.checksum_error = True
        self.temp_packet_data.packet_available = False

      self.serial_read_state = SerialReadState.INIT








    




    