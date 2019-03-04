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

	joint0 = np.array([None])
	joint1 = np.array([None])
	joint2 = np.array([None])
	joint3 = np.array([None])
	joint4 = np.array([None])
	joint5 = np.array([None])
	
	storedPos = np.array([joint0,joint1,joint2,joint3,joint4,joint5])


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
		thread.start_new_thread(self.freedrive,(False,))
		while self.teachModeBool:
			isStore = raw_input("Type something to store position") 
			while not isStore == None:
				self.storedPos = self.storeCurrentPosition
				print self.storedPos

	def move2TeachedPos(self):
		thread.start_new_thread(self.move2TeachedPos)

		while self.isTeachedPos:
			if not self.storedPos == None:	
				self.main.robotTalk(self.r.move(self.get_stored_positions))
				self.r.waitForMove(0.001,self.get_stored_positions)
				#Do stuff here with the robot
	
	
	def storeCurrentPosition(self,sleeptime):
		joint0=np.array([0])
		joint1=np.array([0])
		joint2=np.array([0])
		joint3=np.array([0])
		joint4=np.array([0])
		joint5=np.array([0])
		while self.runSCP==True:
			joint0=np.append(joint0,[self.r.getCurrentPosition[0]])
			joint1=np.append(joint1,[self.r.getCurrentPosition[1]])
			joint2=np.append(joint2,[self.r.getCurrentPosition[2]])
			joint3=np.append(joint3,[self.r.getCurrentPosition[3]])
			joint4=np.append(joint4,[self.r.getCurrentPosition[4]])
			joint5=np.append(joint5,[self.r.getCurrentPosition[5]])
			time.sleep(sleeptime)
		dataPoints=np.array([joint0,joint1,joint2,joint3,joint4,joint5])
		return dataPoints
	
	def stopSCP(self):
		self.runSCP=False

	# Access to the stored postions after completing teaching mode 
	def get_stored_positions(self):
			return self.storedPos	
	

	
	def setMove2posbool(self,bool):
		self.move2pobool=bool
	def setfreedrivebool(self,bool):
		self.freedrivebool=bool
	def setTeachModeBool(self, bool):
		self.teachModeBool=bool
	def set_isTeachedPos_Bool(self, bool):
		self.isTeachedPos = bool


			
	
		
