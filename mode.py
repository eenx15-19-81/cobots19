#!/usr/bin/env python
import rospy
import time
import math

class mode():
	joint_home=[0,-math.pi/2,0,-math.pi/2, 0, 0]
	joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	posOverCube=[-0.373, -1.536, -2.199, -0.966, 1.537, -0.444]
	posAtCube=[-0.373, -1.690, -2.298, -0.680, 1.530,-0.443]
	move2pobool=True
	freedrivebool=True
	def __init__(self,r,g,o,main):
		self.r=r
		self.g=g
		self.o=o
		self.main=main
	# pos over cube [-0.373, -1,536, -2.199, -0.966, 1,537, -0.444]
	# pos at cube [-0.373, -1.690, -2.298, -0.680, 1,530,-0.443]
	def move2pos(self):
		 while self.move2pobool:
			self.main.robotTalk(self.r.move(self.joint_home))
			self.r.waitForMove(0.001,self.joint_home)
			self.main.robotTalk(self.r.move(self.posOverCube))
			self.r.waitForMove(0.001,self.posOverCube)
			self.main.gripperTalk(self.g.open())
			self.main.robotTalk(self.r.move(self.posAtCube))
			self.r.waitForMove(0.001,self.posAtCube)
			self.main.gripperTalk(self.g.close())
			self.main.robotTalk(self.r.move(self.posOverCube))
			self.r.waitForMove(0.001,self.posOverCube)
			self.main.robotTalk(self.r.move(self.joint_home))
			self.r.waitForMove(0.001,self.joint_home)
			time.sleep(5)
			self.main.robotTalk(self.r.move(self.posOverCube))
			self.r.waitForMove(0.001,self.posOverCube)
			time.sleep(0.5)
			self.main.robotTalk(self.r.move(self.posAtCube))
			self.r.waitForMove(0.001,self.posAtCube)
			self.main.gripperTalk(self.g.open())
			self.main.robotTalk(self.r.move(self.posOverCube))
			self.r.waitForMove(0.001,self.posOverCube)
	def freedrive(self):

		if self.freedrivebool:
			#self.main.robotTalk(self.r.move(self.joint_pose2))
			#self.r.waitForMove(0.001, self.joint_pose2)
			time.sleep(1)
			print("WARNING, robot will move.\n")
			time.sleep(3)
			movePos=self.r.move(self.o.calibration())
			print(movePos)
			self.main.robotTalk(movePos)
			self.r.waitForMove(0.001,self.o.calibration())
			time.sleep(2)
			self.main.optoZeroPub.publish(True)
			time.sleep(2)
			print("ready to move")
			while self.freedrivebool:
				self.main.robotTalk(self.o.getSpeedl())
				self.main.rate.sleep() 
			self.main.robotTalk("stopl(1) \n") 






	def setMove2posbool(self,bool):
		self.move2pobool=bool
	def setfreedrivebool(self,bool):
		self.freedrivebool=bool
		