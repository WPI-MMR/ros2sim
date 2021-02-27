from ros2sim.parsers import special as s
import serial


if __name__ == '__main__':
  port = input('Enter the port to bind to: ')
  ser = serial.Serial(port, 9600, timeout=1)

  try:
    while True:
      # for i in range(21):
      #   cmd = int(input('Send the following command: '))
      #   ser.write(bytes([cmd]))# + s.EOM.value)
    
      cmd = int(input('Press key to start test case: '))
      # test_case = [255, 255, 255, 255, 1, 0, 2, 0, 3, 0, 4,
      # 0, 5, 0, 6, 0, 7, 0, 8, 0, 219]

      test_case = [255, 255, 255, 255, 45, 0, 90, 0, 45, 0, 90,
      0, 45, 0, 90, 0, 45, 0, 90, 0, 227]

      if cmd == 1:
        for val in test_case:
          ser.write(bytes([val]))
      elif cmd == 2:
        inp = [255, 255, 255, 255, 1, 1, 253]
        for i in inp:
          ser.write(bytes([i]))
      elif cmd == 3:
        if ser.in_waiting:
          for i in range(28):
            received_data = ser.read()
            print(int(received_data.hex(), 16))
      # res = b''
      # while not res.endswith(s.EOM.value):
      #   res += ser.read()
      
      # print('Response: {}'.format(res))

  except KeyboardInterrupt:
    pass