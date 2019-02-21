#!/usr/bin/env python
# Listeners

class robotListener():
	currentPosition = []
	def __init__(self):
		rospy.Subscriber("/joint_states",JointState,self.callback)

	# Callback from the URSubscriber updating jointstates with current position
	def callback(self,data):
		self.currentPosition = data.position
		
	def getCurrentPosition(self):
		return currentPosition
		