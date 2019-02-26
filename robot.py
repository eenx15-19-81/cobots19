#!/usr/bin/env python
# Robot

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState

class robot():
	currentPosition=[1,1,1,1,1,1]
	acceleration=0.1
	velocity=0.3
	def __init__(self):
		joint_home=[0,-1.5,0,-1.5, 0, 0]
		joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	
		print "Initializing robot."
		rospy.init_node('robot',anonymous=True)
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		self.commandPub=rospy.Publisher('/robotCommand',String,queue_size=10)
		rospy.Subscriber("/joint_states",JointState,self.callback)
		rospy.Subscriber("/robotCommand",String,self.callbackCommand)
		print "Succesfully initialized robot."

	#	while not rospy.is_shutdown():
	#		print("Going to Home")
	#		self.move(joint_home)
	#		self.waitForMove(0.001, joint_home)
	#		self.commandTalk("close")
	#		time.sleep(1)
	#		self.commandTalk("open")
	#		time.sleep(1)
	#		print("Going to Pos")
	#		self.move(joint_pose2)
	#		self.waitForMove(0.001, joint_pose2)
		while not rospy.is_shutdown():
			self.commandPub.publish("Hello my hand")
			time.sleep(1)
	
	
	def move(self,pos):
		move = "movej("+str(pos)+",a="+str(self.acceleration)+",v="+str(self.velocity)+",t="+str(0)+",r="+str(0) +")"
		self.robotTalk(move)
	
	# Wait for current move to be done [Margin in radians, Desired position as 6 floats of radians]
	def waitForMove(self, margin, desiredPosition):
		print("Waiting for move...")
		done = False
		while True:
			for x in range(0,6):
				if(abs(desiredPosition[x] - self.currentPosition[x]) > margin):
					done = False
				else:
					done = True
			if(done == True):
				break	

	# Callback from the URSubscriber updating jointstates with current position
	def callback(self,data):
		self.currentPosition = data.position
	# Callback from the robotCommand
	def callbackCommand(self,data):
		s=data.data.lower()
		if s=='done':
			print "hehe"
		elif s=='something else':
			print "hej"
	# Publish publishing messages to topics
	def robotTalk(self,msg):
		self.urPublisher.publish(msg)
	def commandTalk(self,msg):
		self.commandPub.publish(msg)

try:
	robot()
except rospy.ROSInterruptException:
	pass
