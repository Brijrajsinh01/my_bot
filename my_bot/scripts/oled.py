import serial
import cv2
import numpy as np
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with your Arduino's serial port
data=1
if data==1:
    print("test")
    ser.write(b'MOVE_CLOCKWISE')
elif data==0:
    ser.write(b'STOP')

try:
    # ser = serial.Serial('COM5', 9600)  # Use 'COM5' or your actual port name
    print("Connection to Arduino established.")
except serial.SerialException:
    print("Failed to establish a connection to Arduino. Check the COM port.")

# Read and write data to the Arduino
while True:
    
    try:
        data = ser.readline().decode().strip()
        print("truuuuuuuu")
        print(f"Arduino says: {data}")
        # Your code for sending commands to the Arduino
    except serial.SerialException:
        print("Lost connection to Arduino. Reconnecting...")
        # You can add code here to attempt to reconnect or take appropriate action.
