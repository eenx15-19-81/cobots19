#!/usr/bin/env python
# GripperTalker

import rospy
import time
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class gripper():
	# Values for the standard open and the close messages.
	closeGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 0)
	openGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 0)
	
	# Current status of the gripper.
	currentACT = 0
	currentGTO = 0
	currentATR = 0
	currentPR = 0
	currentSP = 0
	currentFR = 0
		
	def __init__(self):
		print "Gripper connected"
		
	# Updates the gripper class with the latest status from the gripper.
	# Input: the data that is published on the gripper topic.
	def updateStatus(self, data):
		self.currentACT = data.gACT
		self.currentGTO = data.gGTO
		self.currentATR = data.gATR
		self.currentPR = data.gPR
		self.currentSP = data.gSP
		self.currentFR = data.gFR
	
	# Activating gripper
	# Sending first a reset to the gripper so if it have been wrongly activated before it will be a fresh start when our init runs.
	# Sleep 0.1s and then start the activating sequence.
	def activateGripper(self, main):
		msgReset = outputMsg.Robotiq2FGripper_robot_output()
		msgReset.rACT = 0
		main.gripperPub.publish(msgReset)
		time.sleep(0.3)
		msgActivate = outputMsg.Robotiq2FGripper_robot_output()
		msgActivate.rACT=1
		msgActivate.rGTO=1
		msgActivate.rSP=255
		msgActivate.rFR=10
		main.gripperPub.publish(msgActivate)
		time.sleep(1)
	
	######################################
	############# Messages ###############
	######################################
	
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



