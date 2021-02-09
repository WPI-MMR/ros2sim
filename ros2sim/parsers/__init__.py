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
    self.set_to_default()

  def set_to_default(self):
    """Sets the object to the default values
    """
    self.left_hip = 0
    self.left_knee = 0
    self.right_hip = 0
    self.right_knee = 0
    self.left_shoulder = 0
    self.left_elbow = 0
    self.right_shoulder = 0
    self.checksum = 0
    self.checksum_error = False
    self.packet_available = False
    self.data_request = False

  def __str__(self):
    return 'left_hip: {}\nleft_knee: {}\nright_hip: {}\n right_knee: {}\n'\
    'left_shoulder: {}\nleft_elbow: {}\nright_shoulder: {}\n' \
    'checsum: {}\nchecksum_error: {}\npacket_available: {}\n' \
    'data_request: {}'.format(self.left_hip, self.left_knee, self.right_hip,
    self.right_knee, self.left_shoulder, self.left_elbow, self.right_shoulder,
    self.checksum, self.checksum_error, self.packet_available, self.data_request)


from .base_parser import Parser
from.json_parser import JsonParser