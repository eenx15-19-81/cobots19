#!/usr/bin/env python
# GripperTalker

import rospy
import time
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class gripper():
	# Hardcoded values for the standard open and the close messages.
	closeGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 0)
	openGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 0)
		
	def __init__(self):
		print "Gripper initialized"
	# Open gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25
	# Close gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25
	
	# Creates a custom message (msg) for the gripper
	# Input: rPR is the applied preassure between 0 and 255
	# Input: rSP is the speed to opne/close the gripper between 0 and 255
	# Input: rFR 
	def customGrip(self,rAct = 1, rGTO = 1, rATR = 0, rPR, rSP, rFR):
		return outputMsg.Robotiq2FGripper_robot_output(rACT, rGTO, rATR, rPR, rSP, rFR)
		
	# Creates a standard message (msg) to open to gripper.
	# Return: msg (Robotiq)
	def open(self):
		return self.openGrip

	# Creates a standard message (msg) to close to gripper.
	# Return: msg (Robotiq)
	def close(self):
		return self.closeGrip



