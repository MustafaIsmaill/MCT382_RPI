# left analog  --> axis 0 = x value ... axis 1 = y value
# right analog --> axis 2 = x value ... axis 5 = y value

import pygame
from time import sleep
import os

# dummy environment
os.environ["SDL_VIDEODRIVER"] = "dummy"

# setup the pygame window
pygame.init()

# initialise joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

buttons = joystick.get_numbuttons()
hats = joystick.get_numhats()
axes = joystick.get_numaxes()

def getAxis(number):
    # when nothing is moved on an axis, the VALUE IS NOT EXACTLY ZERO
    # so this is used not "if joystick value not zero"
    if joystick.get_axis(number) < -0.1 or joystick.get_axis(number) > 0.1:
      # value between 1.0 and -1.0
      print ("Axis value is %s" %(joystick.get_axis(number)))
      print ("Axis ID is %s" %(number))
 
def getButton(number):
    # returns 1 or 0 - pressed or not
    if joystick.get_button(number):
      # just prints id of button
      print ("Button ID is %s" %(number))

def getHat(number):
    if joystick.get_hat(number) != (0,0):
      # returns tuple with values either 1, 0 or -1
      print ("Hat value is %s, %s" %(joystick.get_hat(number)[0],joystick.get_hat(number)[1]))
      print ("Hat ID is %s" %(number))

while True:
	for event in pygame.event.get():
	# loop through events
		if axes != 0:
			for i in range(axes):
				getAxis(i)
#		if buttons != 0:
#     			for i in range(buttons):
#        			getButton(i)
#		if hats != 0:
#     			for i in range(hats):
#      			getHat(i)
#		print(joystick.get_hat(0)[0], joystick.get_hat(0)[1])
