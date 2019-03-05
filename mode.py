#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
import thread

from subclasses import robot 


class mode():
	joint_home=[0,-math.pi/2,0,-math.pi/2, 0, 0]
	joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	posOverCube=[-0.373, -1.536, -2.199, -0.966, 1.537, -0.444]
	posAtCube=[-0.373, -1.690, -2.298, -0.680, 1.530,-0.443]
	move2pobool=True
	freedrivebool=True
	teachModeBool = True
	isTeachedPos = True
	requestPos=False
	move2TeachedPosBool=True

	joint0 = 0
	joint1 = 0
	joint2 = 0
	joint3 = 0
	joint4 = 0
	joint5 = 0
	
	storedList = []


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
			self.main.robotTalk(self.r.move(self.posOverCube))
			self.r.waitForMove(0.001,self.posOverCube)
			time.sleep(0.5)
			self.main.robotTalk(self.r.move(self.posAtCube))
			self.r.waitForMove(0.001,self.posAtCube)
			self.main.gripperTalk(self.g.open())
			self.main.robotTalk(self.r.move(self.posOverCube))
			self.r.waitForMove(0.001,self.posOverCube)
	
	def freedrive(self,bool):
		while self.freedrivebool:
			time.sleep(1)
			self.main.optoZeroPub.publish(True)
			time.sleep(2)
			while self.freedrivebool:
				self.main.robotTalk(self.o.getSpeedl())
				self.main.rate.sleep() 
			self.main.robotTalk("stopl(1) \n")
	
	def teachmode(self):
		self.storedList=[]
		thread.start_new_thread(self.teachmodethread,(False,))
		while self.freedrivebool:
			time.sleep(1)
			self.main.optoZeroPub.publish(True)
			time.sleep(2)
			print "ready"
			while self.freedrivebool:
				if self.requestPos:
					self.storedList.append(self.storeCurrentPosition())
					print self.storedList
					self.requestPos=False
				self.main.robotTalk(self.o.getSpeedl())
				self.main.rate.sleep() 
			self.main.robotTalk("stopl(1) \n")

	def teachmodethread(self,bool):
		while self.teachModeBool:
			isStore = raw_input("Type something to store position or 'exit' to close")
			if isStore == "exit":
				break
			else:
				self.set_requestPosBool(True)
		thread.exit()


	def move2TeachedPos(self):
		while self.move2TeachedPosBool:
			for x in range (0,len(self.storedList)):
				if type(self.storedList[x]) is list:
					self.main.robotTalk(self.r.move(self.storedList[x]))
					self.r.waitForMove(0.001,self.storedList[x])
				elif type(self.storedList[x]) is str:
					if self.storedList[x] == "Open":
						self.main.gripperTalk(self.g.open())
					elif self.storedList[x] == "Close":
						self.main.gripperTalk(self.g.close())
			self.set_move2TeachedPosBool(False)		
	
	def storeCurrentPosition(self):
		self.joint0=self.r.getCurrentPositionI(0)
		self.joint1=self.r.getCurrentPositionI(1)
		self.joint2=self.r.getCurrentPositionI(2)
		self.joint3=self.r.getCurrentPositionI(3)
		self.joint4=self.r.getCurrentPositionI(4)
		self.joint5=self.r.getCurrentPositionI(5)
		dataPoints=[self.joint0, self.joint1, self.joint2, self.joint3, self.joint4, self.joint5]
		return dataPoints
	
	# Access to the stored postions after completing teaching mode ;
	def get_stored_positions(self):
			return self.storedList
	

	
	def setMove2posbool(self,bool):
		self.move2pobool=bool
	def setfreedrivebool(self,bool):
		self.freedrivebool=bool
	def setTeachModeBool(self, bool):
		self.teachModeBool=bool
	def set_isTeachedPos_Bool(self, bool):
		self.isTeachedPos = bool
	def set_requestPosBool(self,bool):
		self.requestPos=bool
	def set_move2TeachedPosBool(self,bool):
		self.move2TeachedPosBool=bool

			
	
		
