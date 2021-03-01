import serial


if __name__ == '__main__':
  port = input('Enter the port to bind to: ')
  ser = serial.Serial(port, 9600, timeout=1)

  try:
    while True:
    
      cmd = int(input('Press key to start test case: '))

      ## Uncomment to run different test cases. Demonstrates working of the ros2sim package
      # Run the program and enter 1 to execute joint values present in test case on the robot
      # Enter 2 to read the robot state whenever required. It will print the bytes sent my arduino mocker

      # test_case = [255, 255, 255, 255, 0, 45, 0, 90, 0, 45, 0, 90,
      # 0, 45, 0, 90, 0, 45, 0, 90, 0, 227]

      # test_case = [255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0,
      # 0, 0, 0, 0, 0, 0, 0, 0, 0, 255]     

      test_case = [255, 255, 255, 255, 0, 255, 2, 255, 3, 255, 4,
      255, 5, 255, 6, 255, 7, 255, 8, 255, 9, 219]

      if cmd == 1:
        for val in test_case:
          ser.write(bytes([val]))
      elif cmd == 2:
        inp = [255, 255, 255, 255, 1, 254]
        for i in inp:
          ser.write(bytes([i]))

        while (ser.in_waiting == False):
          pass

        for i in range(28):
          received_data = ser.read()
          print(int(received_data.hex(), 16))

  except KeyboardInterrupt:
    pass