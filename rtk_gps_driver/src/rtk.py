import serial

# Replace 'COM3' with your serial port
port = 'COM3'
port = '/dev/ttyACM0'
baudrate = 115200  # Default baudrate for simpleRTK2B

ser = serial.Serial(port, baudrate, timeout=1)

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            print(data)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
