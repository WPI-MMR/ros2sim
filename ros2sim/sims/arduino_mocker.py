import pty
import os
import serial
import threading
import copy
import logging

from ros2sim.parsers import SimExecutor
from ros2sim.parsers import special as s
from ros2sim.parsers import SerialReadState
from ros2sim.parsers import JointInformation


class SerialSimulator:
  def __init__(self, sim_executor: SimExecutor, baudrate=9600):
    """Create a new Serial simulator

    Args:
      sim_executor (SimExecutor): Uses this to get information or make an action on 
      solo_realtime environment
    """
    # Based on the data collected, appropriate actions will be taken using the parser
    self.sim_executor = sim_executor
    self.PREAMBLE_LENGTH = 4
    self.DATA_BYTE_LENGTH = 2

    self.num_joints = 8
    self.imu_axes = 3

    # Packet to store data temporarily as we read it. This data is not validated
    self.temp_packet_data = JointInformation()
    # After validation, data is copied here. For all intents and purposes, this
    # is the only source of verified data
    self.validated_packed_data = JointInformation()
    # Initial state of the state machine
    self.serial_read_state = SerialReadState.INIT
    # List of all possible states for the state machine
    self.list_of_states = [state for state in SerialReadState]

    # These are counter variables to be used in the state machine

    # Running checksum used to determine the validity of the incoming packet.
    # We don't expect the mocking serial to have issues with the packed validity
    # but this is being implemented to keep with consistency of the Arduino code
    self.calculated_checksum = 0
    # Counter to check for the number of ints in preamble
    self.preamble_counter = self.PREAMBLE_LENGTH
    # Counter to check the number of for each joint angle
    self.data_byte_counter = self.DATA_BYTE_LENGTH   

  def serve(self):
    self.master, slave = pty.openpty()
    s_name = os.ttyname(slave)

    thread = threading.Thread(target=self.listener)
    thread.start()

    logging.info("""
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
        self.validated_packed_data.packet_available = False
        if self.validated_packed_data.data_request:
          logging.debug("Requesting data")
          observation_packet = self.sim_executor.get_obs()
          self.sensor_data_response(observation_packet)
        else:
          logging.debug("Updating Joint Angles")
          logging.debug("The received data is: {}".format(self.validated_packed_data))
          self.sim_executor.action(self.validated_packed_data)

    # while True:
    #   request = b''
    #   while not request.endswith(s.EOM.value):
    #     request += os.read(self.master, 1)

    #   response = self.parser.parse(request)
    #   os.write(self.master, response + s.EOM.value)

  def sensor_data_response(self, observation_packet: JointInformation):
    for i in range(self.PREAMBLE_LENGTH):
      os.write(self.master, bytes([255]))

    robot_state = observation_packet.get_robot_state()
    states = list(robot_state.keys())
    raw_sum = 0
    for state in states[:-1]:
      value = robot_state[state]
      raw_sum += value
      val_1 = 255 if value // 256 > 0 else value
      val_2 = value % 255 if value // 256 > 0 else 0
      os.write(self.master, bytes([val_1]))
      os.write(self.master, bytes([val_2]))
    
    # At goal is only one byte
    value = int(robot_state[states[-1]])
    raw_sum += value
    os.write(self.master, bytes([value]))

    checksum = 255 - (raw_sum % 256)
    os.write(self.master, bytes([checksum]))


  def recv_data(self):
    """Checks to see if there is any data over the serial port to read. If there is,
    this method reads it and fills up the corresponding data structures. 
    """

    # TODO: Andrew what is the range of data being transferred.

    if self.serial_read_state == SerialReadState.INIT:
      logging.debug("In INIT")
      # Running checksum used to determine the validity of the incoming packet.
      # We don't expect the mocking serial to have issues with the packed validity
      # but this is being implemented to keep with consistency of the Arduino code
      self.calculated_checksum = 0
      # Counter to check for the number of ints in preamble
      self.preamble_counter = self.PREAMBLE_LENGTH
      # Counter to check the number of for each joint angle
      self.data_byte_counter = self.DATA_BYTE_LENGTH
      # Reset the temp packet to store values in. We decide after reading checksum if the data
      # received is valid
      self.temp_packet_data.set_to_default()
      # Changing serial read state to read preamble
      self.serial_read_state = SerialReadState.READ_PREAMBLE

    # Data that is going to be read of serial. This data is read as BYTES 
    # This is going to block here untill it reads.
    # Arduino has a non blocking version of this same code.
    # TODO: Evaluate if it can be made non blocking
    received_data = os.read(self.master, 1)
    # Converting the data from bytes to int
    received_data = int(received_data.hex(), 16)
    logging.debug("The recieved_data is: {}".format(received_data))

    if self.serial_read_state == SerialReadState.READ_PREAMBLE:
      if received_data == 255:
        logging.debug("In READ_PREAMBLE with preamble counter: {}".format(self.preamble_counter))
        self.preamble_counter -= 1
        if self.preamble_counter == 0:
          self.preamble_counter = self.PREAMBLE_LENGTH
          self.serial_read_state = SerialReadState.READ_L_HIP
      else:
        # TODO: Ask Andrew why do we even need this. If it is corrupt,
        # shouldn't we discard it?
        self.preamble_counter = self.PREAMBLE_LENGTH
        logging.debug("In READ_PREAMBLE and resetting preamble counter: {}".format(self.preamble_counter))

    elif self.serial_read_state == SerialReadState.READ_L_HIP:
      # TODO: Ask Andrew about this condition. Also why couldn't we add a extra byte in the protocol.
      # The request is to get back the robot state
      if self.data_byte_counter == 1 and self.temp_packet_data.left_hip == received_data:
        self.temp_packet_data.data_request = True
        self.temp_packet_data.left_hip += received_data
        self.calculated_checksum += received_data
        self.data_byte_counter = self.DATA_BYTE_LENGTH
        self.serial_read_state = SerialReadState.READ_CHECKSUM
        logging.debug("In READ_L_HIP and received a request to get sensor data")
      else:
        logging.debug("In READ_L_HIP with data_byte_counter: {}".format(self.data_byte_counter))
        self.temp_packet_data.left_hip += received_data
        self.calculated_checksum += received_data
        self.data_byte_counter -= 1
        if self.data_byte_counter == 0:
          logging.debug("In {} with joint angle: {}".format(self.serial_read_state.name, self.temp_packet_data.left_hip)) 
          self.data_byte_counter = self.DATA_BYTE_LENGTH
          self.serial_read_state = SerialReadState.READ_L_KNEE
          
    # Reading all the joint states except READ_L_HIP
    elif self.serial_read_state in self.list_of_states[3:10]:
      logging.debug("In {} with data_byte_counter: {}".format(self.serial_read_state.name, self.data_byte_counter))
      val = self.temp_packet_data.get_joint_value_from_state(self.serial_read_state)
      val += received_data
      self.temp_packet_data.set_joint_value_from_state(self.serial_read_state, val)

      self.calculated_checksum += received_data
      self.data_byte_counter -= 1
      if self.data_byte_counter == 0:
        logging.debug("In {} with joint angle: {}".format(self.serial_read_state.name, val))
        self.data_byte_counter=self.DATA_BYTE_LENGTH
        self.serial_read_state = self.list_of_states[self.list_of_states.index(self.serial_read_state) + 1]
        

    elif self.serial_read_state == SerialReadState.READ_CHECKSUM:
      self.temp_packet_data.checksum = received_data
      logging.debug("Received checksum: {}".format(self.temp_packet_data.checksum))
      logging.debug("Calculated checksum: {}".format(255 - (self.calculated_checksum % 256)))
      if self.temp_packet_data.checksum == 255 - (self.calculated_checksum % 256):
        # This is a good packet

        # TODO: I don't think we need this here if we go to INIT next
        self.calculated_checksum = 0
        self.preamble_counter = self.PREAMBLE_LENGTH

        self.temp_packet_data.checksum_error = False
        self.temp_packet_data.packet_available = True

        self.validated_packed_data = copy.deepcopy(self.temp_packet_data)
        logging.debug("The checksum matches")
      else:
        self.temp_packet_data.checksum_error = True
        self.temp_packet_data.packet_available = False
        logging.error("The checksum does not match")

      self.serial_read_state = SerialReadState.INIT








    




    