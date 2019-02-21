# RobotTalker

import time
#import roslib; roslib.load_manifest('ur_driver')
#import rospy
#import actionlib
#from std_msgs.msg import String

class robotTalker():
	def __init__(self):
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		rospy.init_node('talker',anonymous=True)

	def move(self):
		move = "movej(" + str(pos) + ", a=" + str(self.acceleration) + ", v=" + str(self.velocity) + ", t=" + str(0) + ", r=" + str(0) + ")"
		self.robotTalk(move)
			
	# Publish the msg to UR node
	def robotTalk(self,msg):
		self.urPublisher.publish(msg)
	