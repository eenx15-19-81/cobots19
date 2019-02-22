#!/usr/bin/env python
# GripperTalker

import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class gripper():
	closeGripper = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 0)
	openGripper = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 0)
		
	# Open gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25
	# Close gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25
	def __init__(self):
		print "Initializing gripper"
		try:
			self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
			rospy.init_node('GripperTalker',anonymous=True)
			
			### Activating gripper ###
			msgReset = outputMsg.Robotiq2FGripper_robot_output()
			msgReset.rACT = 0
			self.gripperPub.publish(msgReset)	# Send request to reset gripper
			time.sleep(0.1)
			msgActivate = outputMsg.Robotiq2FGripper_robot_output()
			msgActivate.rACT=1
			msgActivate.rGTO=1
			msgActivate.rSP=255
			msgActivate.rFR=10
			self.gripperPub.publish(msgActivate)	# Send request to activate the gripper
			time.sleep(1)
			print "Succesfully initialized and activated gripper"
		except rospy.ROSInterruptException
			pass
		
	# Publish the msg to Gripper node
	def gripperTalk(self, msg):
		self.gripperPub.publish(msg)	
	def openGripper(self):
		self.gripperTalk(openGripper)
	def closeGripper(self):
		self.gripperTalk(closeGripper)