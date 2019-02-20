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
#from Robotiq2FGripper_robot_output.msg import *


class test():
	JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
	jointStates = []
			   
	def __init__(self):
		print("Init talk")
		self.initTalker()
		print("init sub")
		self.initSubscriber()
		time.sleep(3)
		print("Done with init.")
		a=0.1
		v=0.3
		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
		pos1 = "movej(" + str(joint_home) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(0) + ", r=" + str(0) + ")"
		pos2 = "movej(" + str(joint_pose2) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(0) + ", r=" + str(0) + ")"
		
		while not rospy.is_shutdown():
			print("Going to Home")
			self.talk(pos1)
			self.waitForMove(0.001, joint_home)
			print("Going to Pos")
			self.talk(pos2)
			self.waitForMove(0.001, joint_pose2)

	##################
	###### INIT ######
	##################
	def initTalker(self):
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		rospy.init_node('talker',anonymous=True)
		
	
	def initSubscriber(self):
		#rospy.init_node('listener',anonymous=True)
		rospy.Subscriber("/joint_states",JointState,self.callback)
		#rospy.spin()

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

	
	def callback(self,data):
		print "i Heard: " + str(data.position)
		self.jointStates = data.position
		#time.sleep(0.25)

	
	def talk(self,msg):			
		rospy.loginfo(msg)
		self.urPublisher.publish(msg)									#Publish hello_str to node
		
try:
	test()
except rospy.ROSInterruptException:
	pass
