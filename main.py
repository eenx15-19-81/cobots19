#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import math

from nodes.robot import robot
from nodes.gripper import gripper



class main():
	
	currentPosition = []
	acceleration=0.1
	velocity=0.3
	
	
			   
	def __init__(self):

		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
		self.r = robot()
		self.g = gripper()
	
		# Main working loop

		while True:
			print("Going to Home")
			r.move(joint_home)
			self.waitForMove(0.001, joint_home)
			g.closeGripper()
			time.sleep(1)
			g.openGripper
			time.sleep(1)
			print("Going to Pos")
			r.move(joint_pose2)
			self.waitForMove(0.001, joint_pose2)

		
		
	#####################
	##### Functions #####
	#####################
	
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	def waitForMove(self, margin, desiredPosition):
		print("Waiting for move...")
		done = False
		while True:
			self.r.getCurrentPosition
			for x in range(0,6):
				if(abs(desiredPosition[x] - self.currentPosition[x]) > margin):
					done = False
				else:
					done = True
			if(done == True):
				break	
if __name__=='__main__':
	main()
	
