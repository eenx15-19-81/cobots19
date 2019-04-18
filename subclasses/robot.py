#!/usr/bin/env python
# Robot

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import time
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import JointState

class robot():

	#Initializing a position
	currentPosition=[1,1,1,1,1,1]
	runStoreCurrentPosition=True

	#prints a message to see that the robot is successfully initialized.
	def __init__(self):
		print "Robot initialized"

	# Moves the robot.
	# Input: pos, 6 positional float[] with the format of [x, y, z, rot_x, rot_y, rot_z]
	# Input: acceleration in m/s 
	# Input: velocity, maximum velocity in m/s
	# Input: SHOULD IT BE TIME HERE??!??!?!??!
	# Input: radius of the movement between the points (correct) !?!?!?!??!
	# Returns: String (A command message to move the robot in form of a string)
	def getMoveMessage(self, pos, acceleration = 0.1, velocity = 0.1, time = 0, radius = 0):
		return "movel(p"+str(pos)+",a="+str(acceleration)+",v="+str(velocity)+",t="+str(time)+",r="+str(radius) +")"
	
	# Generates a speedlCommand to move in a certain direction with a set acceleration.
	# Input: velocity as np.array([6 floats])
	def getSpeedlCommand(self, velocity, acceleration = 0.5, time = 0):
		return "speedl(" + np.array2string(velocity, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")"
	
	# Wait for current move to be done [Margin, Desired position as 6 floats of radians]
	# Input: margin, the margin of error in the position
	# Input: desiredPosition as a positional float[] of 6 variables on the same format as currentPosition
	# Input: numberOfIndices
	def waitForMove(self, margin, desiredPosition, numberOfIndices=6):
		done = False
		while not done:
			for x in range(0,numberOfIndices):
				if(abs(desiredPosition[x] - self.currentPosition[x]) > margin):
					done = False
					break
				else:
					done = True
	
	# Updates the current position.
	# Input: current position of the robot from the tf package, a float[] of 6 variables
	def setCurrentPosition(self,curpos):
		self.currentPosition=curpos
	
	# Returns the current position.
	# Return: gets the currentPosition as a float[] with 6 variables. 
	def getCurrentPosition(self):
		return self.currentPosition
	
	# Returns a specific position value based of an index.
	# Input: index in the currentPosition array, a value between 0 and 5.
	# Return: the value at index location in currentPosition array as a float.
	def getCurrentPositionOfIndex(self,index):
		return self.currentPosition[index]