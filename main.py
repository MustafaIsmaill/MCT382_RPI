# left analog  --> axis 1 = y value
# right analog --> axis 3 = x value

import cv2
import numpy as np
import thread
from time import sleep
import serial
import pygame
import os
import RPi.GPIO as GPIO
import struct

ser = serial.Serial(
               port='/dev/ttyACM0',
               baudrate = 115200,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=1
           )

# dummy environment
os.environ["SDL_VIDEODRIVER"] = "dummy"

#loop until a joystick is found
while True:
	try:
		# setup the pygame window
		pygame.init()
		# initialise joystick
		joystick = pygame.joystick.Joystick(0)
		print("connected to DS4")
		break
	except:
		pygame.quit()
		print("no joystick found")
		sleep(5)

#initialise joystick
joystick.init()

#led pins
ledPinR = 37
ledPinL = 35
#set led pins as output
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ledPinR, GPIO.OUT)
GPIO.setup(ledPinL, GPIO.OUT)
#set leds as low
GPIO.output(ledPinR, GPIO.LOW)
GPIO.output(ledPinL, GPIO.LOW)

b = 0 #lights state
m = 0 #mode button
x1 = 0 #joystick hat
y1 = 0 #joystick hat
x2 = 0 #joystick hat
y2 = 0 #joystick hat
l = 0 #left analog
r = 0 #right analog

analog_val = '5\n' #used to send constant pwm in constant speed mode
rotate_val = '9\n' #used to send max rotation value

def set_leds():
	if GPIO.input(ledPinR) == 0: #leds on
		GPIO.output(ledPinR, GPIO.HIGH)
		GPIO.output(ledPinL, GPIO.HIGH)
	elif GPIO.input(ledPinR) == 1: #leds off
		GPIO.output(ledPinR, GPIO.LOW)
		GPIO.output(ledPinL, GPIO.LOW)

def move(x,y,l,r):
	#analog mode
	if (x==0) and (y==0):
		if r > 0: #move right
			ser.write(right)
			ser.write(rotate_val)
		elif r < 0: #move left
			ser.write(left)
			ser.write(rotate_val)
		elif l > 0: #move forward
			l = str(l) #type cast analog value to char
			l += '\n'
			ser.write(forward)
			ser.write(l)
		elif l < 0: #move backward
			l *= -1
			l = str(l) #type cast analog value to char
			l += '\n'
			ser.write(backward)
			ser.write(l)
		else:
			ser.write(stop)
			ser.write(stop)

	#constant speed mode
	else:
		#forward
		if (x==0) and (y==1):
			ser.write(forward)
			ser.write(analog_val)
		#backward
		elif (x==0) and (y==-1):
			ser.write(backward)
			ser.write(analog_val)
		#right
		elif (x==1) and (y==0):
			ser.write(right)
			ser.write(analog_val)
		#left
		elif (x==-1) and (y==0):
			ser.write(left)
			ser.write(analog_val)
	sleep(0.01)

#direction string
forward = 'f\n'
backward = 'b\n'
right = 'r\n'
left = 'l\n'
stop = 's\n'

#dilation and erosion kernels
kernelOpen = np.ones((5, 5))
kernelClose = np.ones((20, 20))

#video camera object
cam = cv2.VideoCapture(0)

#mode 1 = normal RC, mode -1= feature tracking
mode_state = 1

# define limits for red color range we want
redLower = np.array([0, 50, 80])
redUpper = np.array([10, 255, 255])
reddishLower = np.array([170, 80, 70])
reddishUpper = np.array([255, 255, 220])

#camera initial values
initial_area = 0
area = 0
frame_counter = -1

while True:
	for event in pygame.event.get():
		l = joystick.get_axis(1)
		r = joystick.get_axis(3)
		x1 = joystick.get_hat(0)[0]
		y1 = joystick.get_hat(0)[1]
		b = joystick.get_button(2) #triangle button
		m = joystick.get_button(9) #mode
		#scale analog values
		l *= -10
		r *= 10
		if l == 10:
			l = 9

	#type cast analog float values into integer
	l = int(l)
	r = int(r)

	if b == 1:
		set_leds() #toggle led
		sleep(0.2) #for button debounce
	if m == 1:
		frame_counter = -1
		mode_state *= -1
		sleep(0.2) #for button debounce

	if mode_state == 1:
		move(x1, y1, l, r)

	elif mode_state == -1:
		ret_val, image = cam.read()

		frame_counter += 1

		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		redthreshold = cv2.inRange(hsv, redLower, redUpper)
		reddishthreshold = cv2.inRange(hsv, reddishLower, reddishUpper)

		# red has hues between 0-10 and there are pinkish hues in the 200s so we combine both
		output = cv2.bitwise_or(redthreshold, reddishthreshold)

		#dilate and erode
		output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernelOpen)
		output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernelClose)

		cnts, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

		if len(cnts) != 0:

			greatest_cnt = max(cnts, key = cv2.contourArea)

			area = cv2.contourArea(greatest_cnt)

			if frame_counter == 0:
				initial_area = area

			if area < initial_area:
				move(0, 0, 5, 0) #move forward with half speed
				print("moving forward")
			else:
				move(0, 0, 0, 0) #stop
				print("stop")

			print(area, initial_area)
