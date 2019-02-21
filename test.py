#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import math
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg


class test():
	
	jointStates = []
	acceleration=0.1
	velocity=0.3
			   
	def __init__(self):

		self.initialize()
		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	
		closeGripper = outputMsg.Robotiq2FGripper_robot_output(rPR = 255)
		openGripper = outputMsg.Robotiq2FGripper_robot_output(rPR = 0)
		
		# Open gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25
		# Close gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25
	
		# Main working loop

		while not rospy.is_shutdown():
			print("Going to Home")
			self.move(pos1)
			self.waitForMove(0.001, joint_home)
			self.talkGripper(closeGripper)
			time.sleep(1)
			self.talkGripper(openGripper)
			time.sleep(1)
			print("Going to Pos")
			self.move(pos2)
			self.waitForMove(0.001, joint_pose2)

	##################
	###### INIT ######
	##################
	def initTalker(self):
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		rospy.init_node('talker',anonymous=True)
		
	def initTalkerGripper(self):
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
	
	def activateGripper(self):
		self.gripperPub.publish(self.outputMsg.Robotiq2FGripperRobotOutput().rACT=0)	# Send request to reset gripper
		time.sleep(0.1)
		msg = self.outputMsg.Robotiq2FGripperRobotOutput()
		msg.rACT=1
		msg.rGTO=1
		msg.rSP=255
		msg.rFR=10
		self.gripperPub.publish(msg)	# Send request to activate the gripper
		time.sleep(1)
	
	def initSubscriber(self):
		rospy.Subscriber("/joint_states",JointState,self.callback)
		
	def initialize(self):
		print "Initializing Talkers and Subscribers."
		self.initTalker()
		self.initTalkerGripper()
		self.initSubscriber()
		time.sleep(1)
		print("Done with initialization.")
		
		print("Activating the gripper.")
		self.activateGripper()
		print("Gripper activated.")
		
		
		
		
	#####################
	##### Functions #####
	#####################
	
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	def waitForMove(self, margin, desiredPosition):
		print("Waiting for move...")
		done = False
		while not rospy.is_shutdown():
			for x in range(0,6):
				if(abs(desiredPosition[x] - self.jointStates[x]) > margin):
					done = False
				else:
					done = True
			if(done == True):
				break

	def moveit(self,pos):
		move = "movej(" + str(pos) + ", a=" + str(self.acceleration) + ", v=" + str(self.velocity) + ", t=" + str(0) + ", r=" + str(0) + ")"
		talk(move)
		
	# Callback from the URSubscriber updating jointstates with current position
	def callback(self,data):
		self.jointStates = data.position
	
	# Publish the msg to UR node
	def talk(self,msg):			
		rospy.loginfo(msg)
		self.urPublisher.publish(msg)
	
	# Publish the msg to Gripper node
	def talkGripper(self, msg):
		self.gripperPub.publish(msg)
try:
	test()
except rospy.ROSInterruptException:
	pass
