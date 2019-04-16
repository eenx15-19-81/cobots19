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

	#Initializing a position as well as acceleration and velocity.
	currentPosition=[1,1,1,1,1,1]
	acceleration=0.1
	velocity=0.1
	runStoreCurrentPosition=True

	def __init__(self):
		print "Robot initialized"

	# Moves the robot.
	# Input: float[] (Position)
	# Returns: String (A command message to move the robot in form of a string)
	def move(self,pos):
		move = "movel(p"+str(pos)+",a="+str(self.acceleration)+",v="+str(self.velocity)+",t="+str(0)+",r="+str(0) +")"
		return move
	
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	# Input: float, float[]
	def waitForMove(self, margin, desiredPosition, max=6):
		done = False
		while not done:
		#	print "desired position" +str(desiredPosition)
		#	print self.currentPosition
			for x in range(0,max):
				print abs(desiredPosition[x] - self.currentPosition[x])
				if(abs(desiredPosition[x] - self.currentPosition[x]) > margin):
					done = False
					break
				else:
					done = True
	
	# Update the current position.
	# Input: float[]
	def setCurrentPosition(self,curpos):
		#print curpos
		self.currentPosition=curpos
	
	# Returns the current position.
	# Return: float[]
	def getCurrentPosition(self):
		return self.currentPosition
	
	# Returns a specific position value based of an index.
	# Input: int
	# Return: float
	def getCurrentPositionOfIndex(self,index):
		return self.currentPosition[index]

	## Stores currentPosition in n,6 matrix and stops when stopSCP is called
	#def storeCurrentPosition(self,sleepTime):
	#	dataPoints=np.array([[0],[0],[0],[0],[0],[0]])
	#	while self.runStoreCurrentPosition==True:
	#		dataPoints=np.append(dataPoints,[[self.currentPosition[0]],[self.currentPosition[1]],[self.currentPosition[2]],[self.currentPosition[3]],self.currentPosition[4]],[self.currentPosition[5]]])
	#		time.sleep(sleepTime)
	#	return dataPoints
	#def stopSCP(self):
	#	self.runStoreCurrentPosition=False
