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
	currentPosition=[1,1,1,1,1,1]
	acceleration=0.5
	velocity=1.5
	runStoreCurrentPosition=True
	def __init__(self):
		print "Robot initialization"

	def move(self,pos):
		move = "movej("+str(pos)+",a="+str(self.acceleration)+",v="+str(self.velocity)+",t="+str(0)+",r="+str(0) +")"
		return move
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	def waitForMove(self, margin, desiredPosition):
		done = False
		while not done:
			for x in range(0,6):
				if(abs(desiredPosition[x] - self.currentPosition[x]) > margin):
					done = False
					break
				else:
					done = True
	
	def setCurrentPosition(self,curpos):
		self.currentPosition=curpos
	def getCurrentPosition(self):
		return self.currentPosition
	def getCurrentPositionI(self,index):
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
