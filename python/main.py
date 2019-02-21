#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import math

from talkers import robotTalker
from talkers import gripperTalker

from listeners import robotListener
from listeners import gripperListener



class main():
	
	currentPosition = []
	acceleration=0.1
	velocity=0.3
	
	rt=robotTalker()
	gt=gripperTalker()
	rl=robotListener()
	gl=gripperListener()
			   
	def __init__(self):

		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	
		# Main working loop

		while not rospy.is_shutdown():
			print("Going to Home")
			rt.move(joint_home)
			self.waitForMove(0.001, joint_home)
			gt.closeGripper()
			time.sleep(1)
			gt.openGripper
			time.sleep(1)
			print("Going to Pos")
			rt.move(joint_pose2)
			self.waitForMove(0.001, joint_pose2)

		
		
	#####################
	##### Functions #####
	#####################
	
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	def waitForMove(self, margin, desiredPosition):
		print("Waiting for move...")
		done = False
		while not rospy.is_shutdown():
			self.robotListener.getCurrentPosition
			for x in range(0,6):
				if(abs(desiredPosition[x] - self.currentPosition[x]) > margin):
					done = False
				else:
					done = True
			if(done == True):
				break	
try:
	test()
except rospy.ROSInterruptException:
	pass
