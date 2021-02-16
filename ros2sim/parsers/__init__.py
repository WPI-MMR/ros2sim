from enum import Enum, auto
from typing import List


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
    self.set_all_joint_values()
    self.theta_x = 0
    self.theta_y = 0
    self.theta_z = 0
    self.checksum = 0
    self.checksum_error = False
    self.packet_available = False
    self.data_request = False
    self.at_goal = False

  def __str__(self):
    return 'left_hip: {}\nleft_knee: {}\nright_hip: {}\nright_knee: {}\n'\
    'left_shoulder: {}\nleft_elbow: {}\nright_shoulder: {}\n' \
    'checksum: {}\nchecksum_error: {}\npacket_available: {}\n' \
    'data_request: {}'.format(self.left_hip, self.left_knee, self.right_hip,
    self.right_knee, self.left_shoulder, self.left_elbow, self.right_shoulder,
    self.checksum, self.checksum_error, self.packet_available, self.data_request)

  def set_theta_value(self, s: str, val: int):
    """Sets the imu theta value
    """
    if s == "θx":
      self.theta_x = val
    elif s == "θy":
      self.theta_y = val
    elif s == "θz":
      self.theta_z = val
    else:
      raise ValueError("Imu axes name should be one of these: θx, θy, θz")
  
  def get_theta_value(self, s: str) -> int:
    """Returns the imu value corresponding to the axes represented by s
    """
    if s == "θx":
      return self.theta_x
    elif s == "θy":
      return self.theta_y
    elif s == "θz":
      return self.theta_z
    else:
      raise ValueError("Imu axes name should be one of these: θx, θy, θz")   

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

  def set_all_joint_values(self, left_hip=0, left_knee=0, right_hip=0, right_knee=0,
    left_shoulder=0, left_elbow=0, right_shoulder=0, right_elbow=0):
    """Sets the joint angles based on the values of the named arguments
    """
    self.left_hip = left_hip
    self.left_knee = left_knee
    self.right_hip = right_hip
    self.right_knee = right_knee
    self.left_shoulder = left_shoulder
    self.left_elbow = left_elbow
    self.right_shoulder = right_shoulder
    self.right_elbow = right_elbow

  def get_joint_values(self, ):
    """Return a dictionary containing joint values based on joint ordering
    """
    pass

  def get_joint_value_from_state(self, s: SerialReadState) -> int:
    """Returns the joint value of the corresponding SerialReadState
    """
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

  def get_robot_state(self):
    """Returns the state of the robot as a dictionary
    """
    return {
      "theta_x": self.theta_x,
      "theta_y": self.theta_y,
      "theta_z": self.theta_z,
      "l_hip": self.left_hip,
      "l_knee": self.left_knee,
      "r_hip": self.right_hip,
      "r_knee": self.right_knee,
      "l_shoulder": self.left_shoulder,
      "l_elbow": self.left_elbow,
      "r_shoulder": self.right_shoulder,
      "r_elbow": self.right_elbow,
      "at_goal": self.at_goal
    }



from .sim_executor import SimExecutor