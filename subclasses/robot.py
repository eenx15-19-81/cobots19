#!/usr/bin/env python
# Robot

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState

class robot():
	currentPosition=[1,1,1,1,1,1]
	acceleration=0.5
	velocity=1.5
	def __init__(self):
		print "hej"

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
		
