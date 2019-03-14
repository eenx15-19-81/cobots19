#!/usr/bin/env python
# GripperTalker

import rospy
import time
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class gripper():
	# Hardcoded values for the open and the close message.
	closeGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 0)
	openGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 0)
		
	def __init__(self):
		print "Gripper initialized"
	# Open gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25
	# Close gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25

	# Publish msg to open to Gripper node.
	# Return: msg (Robotiq)
	def open(self):
		return self.openGrip

	# Publish msg to close to Gripper node.
	# Return: msg (Robotiq)
	def close(self):
		return self.closeGrip

#	def callbackCommand(self,data):
#		s=data.data.lower()
#		if s=='open':
#			self.openGripper()
#		elif s=='close':
#			self.closeGripper()
#		elif s.find(' ')!=-1:
#			substring=s.split(' ')
#
#			if substring[0]=='adminstring':
#				self.gripperTalk(substring[1]) 
#		



