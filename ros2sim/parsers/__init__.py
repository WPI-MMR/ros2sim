from enum import Enum, auto

class special(Enum):
  EOM = b'\r\n'
  ACTION = b'ACTION'
  RESET = b'RESET'
  OBS_REQUEST = b'OBS'
  OK = b'OK'

class SerialReadState(Enum):
  INIT = auto()
  READ_PREAMBLE = auto()
  READ_L_HIP = auto()
  READ_L_KNEE = auto()
  READ_R_HIP = auto()
  READ_R_KNEE = auto()
  READ_L_SHOULDER = auto()
  READ_L_ELBOW = auto()
  READ_R_SHOULDER = auto()
  READ_R_ELBOW = auto()
  READ_CHECKSUM = auto()

class JointInformation():
  """Contains the joint information received from serial connection
  """
  def __init__(self):
    """Initalized with dummy values
    """
    self.left_hip = 0
    self.left_knee = 0
    self.right_hip = 0
    self.right_knee = 0
    self.left_shoulder = 0
    self.left_elbow = 0
    self.right_shoulder = 0
    self.checksum = 0
    self.checsum_error = False
    self.packet_available = False
    self.data_request = False


from .base_parser import Parser
from.json_parser import JsonParser