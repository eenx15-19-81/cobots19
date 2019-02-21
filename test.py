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
			   
	def __init__(self):
		self.initialize()
		acceleration=0.1
		velocity=0.3
		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
		pos1 = "movej(" + str(joint_home) + ", a=" + str(acceleration) + ", v=" + str(velocity) + ", t=" + str(0) + ", r=" + str(0) + ")"
		pos2 = "movej(" + str(joint_pose2) + ", a=" + str(acceleration) + ", v=" + str(velocity) + ", t=" + str(0) + ", r=" + str(0) + ")"

		closeGripper = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25)
		openGripper = outputMsg.Robotiq2FGripper_robot_output(rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25)
		
		# Open gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 0, rSP = 255, rFR = 25
		# Close gripper: rACT = 1, rGTO = 1, rATR = 0, rPR = 255, rSP = 255, rFR = 25
	
		# Main working loop
		while not rospy.is_shutdown():
			print("Going to Home")
			self.talk(pos1)
			self.waitForMove(0.001, joint_home)
			self.talkGripper(closeGripper)
			time.sleep(1)
			self.talkGripper(openGripper)
			time.sleep(1)
			print("Going to Pos")
			self.talk(pos2)
			self.waitForMove(0.001, joint_pose2)

	##################
	###### INIT ######
	##################
	def initTalker(self):
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		rospy.init_node('talker',anonymous=True)
		
	def initTalkerGripper(self):
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
	
	def initSubscriber(self):
		rospy.Subscriber("/joint_states",JointState,self.callback)
		
	def initialize(self):
		print "Initializing Talkers and Subscribers."
		self.initTalker()
		self.initTalkerGripper()
		self.initSubscriber()
		time.sleep(3)
		print("Done with initialization.")
		
		
	#####################
	##### Functions #####
	#####################
	
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	def waitForMove(self, margin, desiredPosition):
		print("Waiting for move...")
		done = False
		while(True):
			for x in range(0,6):
				if(abs(desiredPosition[x] - self.jointStates[x]) > margin):
					done = False
				else:
					done = True
				#print str(x) + ": " + str(done)
				print str(desiredPosition[x]) + " | " + str(self.jointStates[x])
			if(done == True):
				break
			time.sleep(0.25)
			print("______________________")
	
		
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
