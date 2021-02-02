import enum

class special(enum.Enum):
  EOM = b'\r\n'
  ACTION = b'ACTION'
  RESET = b'RESET'
  OBS_REQUEST = b'OBS'
  OK = b'OK'


from .base_parser import Parser