import time
import serial

ser = serial.Serial(
               port='/dev/ttyACM0',
               baudrate = 115200,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=1
           )
counter = 1
while True:
	msg = 'f\n'

	ser.write(msg.encode())
