#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
import math
from std_msgs.msg import String

import robot 
import gripper

class main():
			   
	def __init__(self):

		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
		self.r=robot.robot()
		self.g=gripper.gripper()

		
		## Initializing 
		rospy.init_node('main',anonymous=True)
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
		rospy.Subscriber("/joint_states",JointState,self.callback)
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

		## working loop
		while not rospy.is_shutdown():
			self.robotTalk(self.r.move(joint_home))
			self.r.waitForMove(0.001, joint_home)
			self.gripperTalk(self.g.open())
			time.sleep(1)
			self.gripperTalk(self.g.close())
			time.sleep(1)
			self.robotTalk(self.r.move(joint_pose2))
			self.r.waitForMove(0.001, joint_pose2)
		

	def gripperTalk(self, msg):
		self.gripperPub.publish(msg)
	# Publish publishing messages to topics
	def robotTalk(self,msg):
		self.urPublisher.publish(msg)
	# Callback from the URSubscriber updating jointstates with current position
	def callback(self,data):
		self.r.setCurrentPosition(data.position)


try:
	main()
except rospy.ROSInterruptException:
	pass

