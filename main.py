# left analog  --> axis 1 = y value
# right analog --> axis 3 = x value

import time
import serial
import pygame
import os

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

#hat values
x=0
y=0

forward = 'f\n'
backward = 'b\n'
right = 'r\n'
left = 'l\n'

keyF = 'n' #indicates north direction
keyB = 's' #indicates south direction
keyR = 'e' #indicates east direction
keyL = 'w' #indicates west direction

end = 'k' #indicates end of string

line = '\n'

l = 0 #left analog
r = 0 #right analog

while True:
	for event in pygame.event.get():
		if hats != 0:
			for i in range(hats):
				x = joystick.get_hat(i)[0]
				y = joystick.get_hat(i)[1]
		if axes != 0:
			l = joystick.get_axis(1)*-100
			r = joystick.get_axis(3)*100
	l = int(l)
	r = int(r)

	if l>0:
		ser.write(keyF.encode())	#write north key
		ser.write(chr(l).encode())	#write north value

		if r>0:
			ser.write(keyR.encode())	#write east key
			ser.write(chr(r).encode())	#write east value
			ser.write(end.encode())		#write end char
			ser.write(line.encode())	#write new line
		elif r<0:
			r = r*-1
			ser.write(keyL.encode())	#write west key
			ser.write(chr(r).encode())	#write west value
			ser.write(end.encode())		#write end char
			ser.write(line.encode())	#write new line
		else:
			ser.write(end.encode())         #write end char
			ser.write(line.encode())        #write new line

	elif l<0:
		l = l*-1
		ser.write(keyB.encode())        #write south key
		ser.write(chr(l).encode())      #write south value

		if r>0:
			ser.write(keyR.encode())        #write east key
			ser.write(chr(r).encode())      #write east value
			ser.write(end.encode())         #write end char
			ser.write(line.encode())        #write new line
		elif r<0:
			r = r*-1
			ser.write(keyL.encode())        #write west key
			ser.write(chr(r).encode())      #write west value
			ser.write(end.encode())         #write end char
			ser.write(line.encode())        #write new line
		else:
			ser.write(end.encode())         #write end char
			ser.write(line.encode())        #write new line

#	print(l,r)

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

#	time.sleep(1)
