# left analog  --> axis 1 = y value
# right analog --> axis 3 = x value

import cv2
import numpy as np
from threading import Thread
import time
import serial
import pygame
import os
import RPi.GPIO as GPIO

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
		time.sleep(5)

#initialise joystick
joystick.init()

#led pins
ledPinR = 35
ledPinL = 37
#set led pins as output
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ledPinR, GPIO.OUT)
GPIO.setup(ledPinL, GPIO.OUT)
#set leds as low
GPIO.output(ledPinR, GPIO.LOW)
GPIO.output(ledPinL, GPIO.LOW)

b = 0 #lights state
m = 0 #mode button
x1 = 0 #hatx
y1 = 0 #haty

x2 = 0 #hatx for image processing mode
y2 = 0 #haty for image processing mode

def set_leds():
	if GPIO.input(ledPinR) == 0: #leds on
		GPIO.output(ledPinR, GPIO.HIGH)
		GPIO.output(ledPinL, GPIO.HIGH)
	elif GPIO.input(ledPinR) == 1: #leds off
		GPIO.output(ledPinR, GPIO.LOW)
		GPIO.output(ledPinL, GPIO.LOW)

def move(x,y):
	#forward
	if (x==0) and (y==1):
		ser.write(forward.encode())
	#backward
	elif (x==0) and (y==-1):
		ser.write(backward.encode())
	#right
	elif (x==1) and (y==0):
		ser.write(right.encode())
	#left
	elif (x==-1) and (y==0):
		ser.write(left.encode())
	#stop
	elif (x==0) and (y==0):
		ser.write(stop.encode())

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

# Define a function for the first thread
def send_pwms():

	global forward, backward, right, left, stop
	global x1, x2, y1, y2, b, m, frame_counter, mode_state

	while True:
		for event in pygame.event.get():
			x1 = joystick.get_hat(0)[0]
			y1 = joystick.get_hat(0)[1]
			b = joystick.get_button(2) #triangle button
			m = joystick.get_button(9) #mode

		if b == 1:
			set_leds() #toggle led
			time.sleep(0.2) #for button debounce
		if m == 1:
			frame_counter = -1
			mode_state *= -1
			time.sleep(0.2) #for button debounce

		if mode_state == 1:
			move(x1,y1)
		elif mode_state == -1:
			move(x2,y2)

# Define a function for the second thread
def find_red_strap():

	global frame_counter, initial_area, area, x2, y2

	while True:
		while mode_state == -1:
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
					x2 = 0
					y2 = 1
					print("moving forward")
				else:
					x2 = 0
					y2 = 0
					print("stop")

				print(area, initial_area)

#create two threads
if __name__ == "__main__":

	t1 = Thread(target = send_pwms)
	t2 = Thread(target = find_red_strap)
	t1.setDaemon(True)
	t2.setDaemon(True)
	t1.start()
	t2.start()

	while True:
#		for event in pygame.event.get():
#			x1 = joystick.get_hat(0)[0]
#			y1 = joystick.get_hat(0)[1]
#			b = joystick.get_button(2) #triangle button
#			m = joystick.get_button(9) #mode

#		if b == 1:
#			set_leds() #toggle led
#			time.sleep(0.2) #for button debounce
#		if m == 1:
#			frame_counter = -1
#			mode_state *= -1
#			time.sleep(0.2) #for button debounce

#		if mode_state == 1:
#			move(x1,y1)
#		elif mode_state == -1:
#			move(x2,y2)
		pass
