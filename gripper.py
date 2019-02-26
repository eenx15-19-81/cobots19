#!/usr/bin/env python
# GripperTalker

import rospy
import time
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

class gripper():
	closeGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 0)
	openGrip = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 0)
		
	# Open gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25
	# Close gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25
	def __init__(self):
		print "Initializing gripper"
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
		self.commandPub = rospy.Publisher('/robotCommand',String,queue_size=10)
		self.gripperCommand = rospy.Subscriber("/robotCommand",String,self.callbackCommand)
		rospy.init_node('Gripper',anonymous=True)
		time.sleep(1)
		
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
		#while not rospy.is_shutdown():
			#self.commandPub.publish("grip")
			#time.sleep(1)
		rospy.spin()
	# Publish the msg to Gripper node
	def gripperTalk(self, msg):
		self.gripperPub.publish(msg)	
	def openGripper(self):
		self.gripperTalk(self.openGrip)
	def closeGripper(self):
		self.gripperTalk(self.closeGrip)

	def callbackCommand(self,data):
		s=data.data.lower()
		if s=='open':
			self.openGripper()
		elif s=='close':
			self.closeGripper()
		elif s.find(' ')!=-1:
			substring=s.split(' ')

			if substring[0]=='adminstring':
				self.gripperTalk(substring[1])
		

try:
	gripper()
except rospy.ROSInterruptException:
	pass


