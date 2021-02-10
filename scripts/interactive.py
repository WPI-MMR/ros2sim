from ros2sim.parsers import special as s
import serial


if __name__ == '__main__':
  port = input('Enter the port to bind to: ')
  ser = serial.Serial(port, 9600, timeout=1)

  try:
    while True:
      for i in range(21):
        cmd = int(input('Send the following command: '))
        ser.write(bytes([cmd]))# + s.EOM.value)

      # res = b''
      # while not res.endswith(s.EOM.value):
      #   res += ser.read()
      
      # print('Response: {}'.format(res))

  except KeyboardInterrupt:
    pass