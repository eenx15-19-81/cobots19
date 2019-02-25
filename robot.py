#!/usr/bin/env python
# Robot

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class robot():
	def __init__(self):
		print "Initializing robot."
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		rospy.init_node('robot',anonymous=True)
		rospy.Subscriber("/joint_states",JointState,self.callback)
		rospy.Subscriber("/robotCommand",String,self.callbackCommand)
		print "Succesfully initialized robot."
		rospy.spin()
		
	# Callback from the URSubscriber updating jointstates with current position
	def callback(self,data):
		self.currentPosition = data.position
	def callbackCommand(self,data):
		print data
	def getCurrentPosition(self):
		return currentPosition	

	def move(self):
		move = "movej(" + str(pos) + ", a=" + str(self.acceleration) + ", v=" + str(self.velocity) + ", t=" + str(0) + ", r=" + str(0) + ")"
		self.robotTalk(move)
			
	# Publish the msg to UR node
	def robotTalk(self,msg):
		self.urPublisher.publish(msg)
	


try:
	robot()
except rospy.ROSInterruptException:
	pass