#!/usr/bin/env python
import rospy
import time

class mode():
	joint_home=[0,-1.5,0,-1.5, 0, 0]
	joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	def __init__(self,r,g,o,main):
		self.r=r
		self.g=g
		self.o=o
		self.main=main
