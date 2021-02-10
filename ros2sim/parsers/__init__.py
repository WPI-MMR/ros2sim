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
    self.right_elbow = 0
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

  def set_joint_value_from_state(self, s: SerialReadState, value: int):
    """Sets the joint value for the given SerialReadState
    """
    if s == SerialReadState.READ_L_HIP:
      self.left_hip = value
    elif s == SerialReadState.READ_L_KNEE:
      self.left_knee = value
    elif s == SerialReadState.READ_R_HIP:
      self.right_hip = value
    elif s == SerialReadState.READ_R_KNEE:
      self.right_knee = value
    elif s == SerialReadState.READ_L_SHOULDER:
      self.left_shoulder = value
    elif s == SerialReadState.READ_L_ELBOW:
      self.left_elbow = value
    elif s == SerialReadState.READ_R_SHOULDER:
      self.right_shoulder = value
    elif s == SerialReadState.READ_R_ELBOW:
      self.right_elbow = value

  def get_joint_value_from_state(self, s: SerialReadState) -> int:
    if s == SerialReadState.READ_L_HIP:
      return self.left_hip
    elif s == SerialReadState.READ_L_KNEE:
      return self.left_knee
    elif s == SerialReadState.READ_R_HIP:
      return self.right_hip
    elif s == SerialReadState.READ_R_KNEE:
      return self.right_knee
    elif s == SerialReadState.READ_L_SHOULDER:
      return self.left_shoulder
    elif s == SerialReadState.READ_L_ELBOW:
      return self.left_elbow
    elif s == SerialReadState.READ_R_SHOULDER:
      return self.right_shoulder
    elif s == SerialReadState.READ_R_ELBOW:
      return self.right_elbow 


from .base_parser import Parser
from.json_parser import JsonParser