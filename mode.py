#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
import thread
import ast

from math import pi

from subclasses import robot 
#from modes import freedrive
#from modes import teachmode
#from modes import executeSeq


class mode():
	# Hardcoded positions that are being used in move2pos or predefined mode.
	jointHome=[0,-math.pi/2,0,-math.pi/2, 0, 0]
	jointPose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	posOverCube=[-0.373, -1.536, -2.199, -0.966, 1.537, -0.444]
	posAtCube=[-0.373, -1.690, -2.298, -0.680, 1.530,-0.443]
	
	# Initialization of all the global booleans to False so that the robot will not go in to a mode by default.
	move2PredefBool = False
	freedriveBool = False
	teachModeBool = False
	isTeachedPos = False
	requestPos = False
	executeSequenceBool = False
	alignBool=False
	sequenceIndex = None

	# Initializing joints.
	joint0 = 0
	joint1 = 0
	joint2 = 0
	joint3 = 0
	joint4 = 0
	joint5 = 0
	
	storedList = []

	tableLength=0.4
	tableWidth=0.3

	# Initializing and creating instances of robot, gripper, optoforce and main
	def __init__(self,r,g,o,main):
		self.r = r
		self.g = g
		self.o = o
		self.main = main

	# Starts up the freedrive mode
	def freedrive(self):
		while self.freedriveBool:
			time.sleep(1)
			self.main.optoZeroPub.publish(True)
			time.sleep(2)
			while self.freedriveBool:
				self.main.robotTalk(self.o.getSpeedl())
				self.main.rate.sleep() 
			self.main.robotTalk("stopl(1) \n")
	
	# Starts upp the teaching mode
	def teachmode(self):
		self.storedList=[]
		time.sleep(1)
		self.main.optoZeroPub.publish(True)
		time.sleep(2)
		print "Ready"
		while self.teachModeBool:
			if self.requestPos:
				self.storedList.append(self.storeCurrentPosition())
				print self.storedList
				self.requestPos=False
			elif self.alignBool:
				self.storedList.append("Align")
				self.storedList.append(self.storeCurrentPosition())
				print self.storedList
				[self.alignOrigo, self.alignAngle]=self.align()
				self.alignBool=False
			self.main.robotTalk(self.o.getSpeedl())
			self.main.rate.sleep() 
		self.main.robotTalk("stopl(1) \n")
		var = raw_input("Save or exit?")
		if var=="save":
			storedSequence=open("storedSequence.txt", "a+")
			storedSequence.write(str(self.storedList))
			storedSequence.write("\n")
			storedSequence.close()
			alignIndices=[]
			alignIndex=0
			self.switchList=[]
			for x in range(0,len(self.storedList)):
				if self.storedList[x] == "Align":
					alignIndices.append(x)
					alignIndex+=1
			if alignIndex != 0:
				for y in range(0,len(alignIndices)):
					if y+1 >= len(alignIndices):
						alignLength = len(self.storedList)
					else:
						alignLength = alignIndices[y+1]
					for z in range(alignIndices[y]+2,alignLength):
						if type(self.storedList[z]) is list:
							if self.storedList[z][0] < self.alignOrigo+self.tableWidth*math.cos(self.alignAngle) and self.storedList[z][0] > self.alignOrigo and self.storedList[z][1] < self.alignOrigo+self.tableLength*math.sin(self.alignAngle) and self.storedList[z][1] > self.alignOrigo:
								self.switchList.append(z)
			print "Done learning"

		print "Exiting to mode selector"

	# Chooses and executes the desired sequenced based on the input.
	# Input: int (Desired sequence number)
	def chooseAndExecuteSeq(self):
	#	while self.executeSequenceBool:
		if not self.sequenceIndex == None:
			sequence = self.getSequence(self.sequenceIndex)
			while self.executeSequenceBool and not self.sequenceIndex == None:
				for x in range (0,len(sequence)):
					if type(sequence[x]) is list:
						self.main.robotTalk(self.r.move(sequence[x]))
						self.r.waitForMove(0.005,sequence[x])
					elif type(sequence[x]) is str:
						if sequence[x] == "Open":
							self.main.gripperTalk(self.g.open())
						elif sequence[x] == "Close":
							self.main.gripperTalk(self.g.close())
						elif sequence[x] == "Align":
							self.main.robotTalk(self.r.move(sequence[x+1]))
							self.r.waitForMove(0.005,sequence[x+1])
							[curOrigo,curAngle]=self.align()
							for y in range(0,len(self.switchList)):
								xx=self.storedList[self.switchList[y]][0]-self.alignOrigo[0]
								yy=self.storedList[self.switchList[y]][1]-self.alignOrigo[1]
								xprim = xx*math.cos(pi/2-curAngle)-yy*math.sin(pi/2-curAngle)+curOrigo[0]
								yprim = xx*math.sin(pi/2-curAngle)+yy*math.cos(pi/2-curAngle)+curOrigo[1]
									
								self.storedList[self.switchList[y]][0] = xprim
								self.storedList[self.switchList[y]][1] = yprim
							x+=1
						else:
							print "There is a fault in the sequence, it contains a string that is not 'Open' or 'Close'"		

	# Stores the current position of the joints in an array and returns that list.
	def storeCurrentPosition(self):
		self.joint0=self.r.getCurrentPositionOfIndex(0)	
		self.joint1=self.r.getCurrentPositionOfIndex(1)
		self.joint2=self.r.getCurrentPositionOfIndex(2)
		self.joint3=self.r.getCurrentPositionOfIndex(3)
		self.joint4=self.r.getCurrentPositionOfIndex(4)
		self.joint5=self.r.getCurrentPositionOfIndex(5)
		dataPoints=[self.joint0, self.joint1, self.joint2, self.joint3, self.joint4, self.joint5]
		return dataPoints

	# Retrives the desired sequence from the text file.
	# Input: int (Desired sequence number)
	def getSequence(self,line):
		storedSequence=open("storedSequence.txt","r+")
		lines=storedSequence.read().splitlines()
		return ast.literal_eval(lines[int(line)])

	# Defines what the buttons will do while in freedrive mode by sending in a msg as an argument. 
	# Button5 exits the mode, the rest does nothing.
	# Input: msg (Button)
	def freedriveButton(self,data):
		if data.button5:
			self.freedriveBool=False
			self.main.setModeSelBool(True)
			print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:4 for saved programs, Button:5 to exit "

	# Defines what the buttons will do while in teaching mode by sending in a msg as an argument. 
	# Button1 saves the position as a waypoint, Button2 open/closes the gripper 
	# depending on what state it is currently in. Button5 exits the mode.
	# Input: msg (Button)
	def teachModeButton(self,data):
		if data.button1:
			self.setRequestPosBool(True)
		elif data.button2:
			if self.main.currentGrippergPR==0:
				self.setRequestPosBool(True)
				time.sleep(0.01)
				self.storedList.append('Close')
				self.main.gripperTalk(self.g.close())
			else:
				self.setRequestPosBool(True)
				time.sleep(0.01)
				self.storedList.append('Open')
				self.main.gripperTalk(self.g.open())
		elif data.button3:
			self.alignBool=True

		elif data.button5:
			print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:4 for saved programs, Button:5 to exit "
			self.teachModeBool=False

	'''TODO: Make it more generic'''
	# Defines what the buttons will do while in choose-and-execute-sequence-mode by sending in a msg as an argument.
	# Button1 runs sequence 0, Button2 runs sequence 1 and so on. Button5 exits the mode.
	# Input: msg (Button) 
	def chooseAndExecuteSeqButton(self, data):
		if data.button1:
			self.sequenceIndex = 0
			print "Program 1 selected"
		elif data.button2:
			self.sequenceIndex = 1
			print "Program 2 selected"
		elif data.button3:
			self.sequenceIndex = 2
			print "Program 3 selected"
		elif data.button4:
			self.sequenceIndex = 3
			print "Program 4 selected"
		elif data.button5:
			if self.sequenceIndex == None:
				print "Exit to mode select"
				self.main.ledPublisher.publish(led1=True,led2=True,led3=True)
				print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:4 for saved programs, Button:5 to exit "
				self.executeSequenceBool = False
				self.main.setModeSelBool(True)
			else:
				print "Exit to sequence selector"
				print "Enter program on button 1,2,3,4 or exit on button 5"
				self.sequenceIndex = None

	# Defines what the buttons will do while in move-to-teached-position mode by sending in a msg as an argument.
	# Button5 exits the mode. 
	# Input: msg (Button)
	#def executeSequenceModeButton(self,data):
	#	if data.button5:
	#		self.executeSequenceBool=False
	#		self.main.setModeSelBool(True)

	# Defines what the buttons will do while in predefined mode by sending in a msg as an argument. Button5 exits the mode. 
	# Input: msg (Button)
	def preDefinedButton(self,data):
		if data.button5:
			self.move2PredefBool=False
			self.main.setModeSelBool(True)
			print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:4 for saved programs, Button:5 to exit "	

	def align(self):
		startPos = self.r.getCurrentPosition()
		self.moveInDirection(np.array([-1,0,0]))
		wallPos1 = self.r.getCurrentPosition()
		self.main.robotTalk(self.r.move(startPos))
		self.r.waitForMove(0.05,startPos,3)
		startPosCopy=[startPos[0],startPos[1]-0.1,startPos[2],startPos[3],startPos[4],startPos[5]]
		self.main.robotTalk(self.r.move(startPosCopy))
		self.r.waitForMove(0.05,startPosCopy,3)
		self.moveInDirection(np.array([-1,0,0]))
		wallPos2 = self.r.getCurrentPosition()
		x = wallPos2[0]-wallPos1[0]
		y = wallPos2[1]-wallPos1[1]
		alpha = math.atan(y/x)
	#	print str(alpha) +" " + str([np.sin(alpha),np.cos(alpha),0])
	#	tmp = self.r.currentPosition
	#	print tmp[4]
	#	tmp[4]=float(tmp[4])+alpha
	#	print tmp[4]
	#	self.main.robotTalk(self.r.move(tmp))
		time.sleep(2)
		self.moveInDirection([np.cos(alpha),np.sin(alpha),0])
		origo = self.r.currentPosition
		print [origo,alpha]
		return [origo,alpha]

		# move in negative x
		# save wall pos 1
		# move to pos 1 negative y
		# move negative x
		# save wall pos 2
		# calculate angle
		# move little bit in pos xpprim
		# rotate tool with angle
		# move to xprim =0
		# move to yprim =0 with speedl
		# collect prim origio

	#Direction as [x,y,z] where x,y,z = 0,-1, 1
	def moveInDirection(self, direction):
		velocity = np.multiply(direction,0.05) 
		velocity=np.concatenate((velocity,np.array([0.0,0.0,0.0])))	
		velocityCommand = self.speedlCommand(velocity)
		while self.checkOpto():
			self.main.robotTalk(velocityCommand)
			self.main.rate.sleep() 
		self.main.robotTalk("stopl(1) \n")

	def checkOpto(self):
		curForce = self.o.getCurForce()
		if curForce[0] > 5 or curForce[1] > 5 or curForce[2] > 5: 
			return False
		else:
			return True

	#velocity as np.array([0.0,0.0,0.0])
	def speedlCommand(self,velocity):
		acceleration = 0.1
		time=0.5
		command = "speedl(" + np.array2string(velocity, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")"
		return command


	# Access to the stored postions after completing teaching mode.
	def getStoredPositions(self):
		return self.storedList

	'''TODO: Ta bort dessa och erstt dom verallt dr dom anvnds med att bara ndra variabeln, den r ju global nd.'''
	# Sets the bool to the value of the bool you are sending in as an argument.
	# Input: True, False
	def setMove2PredefBool(self,bool):
		self.move2PredefBool=bool
	def setfreedriveBool(self,bool):
		self.freedriveBool=bool
	def setTeachModeBool(self, bool):
		self.teachModeBool=bool
	def setIsTeachedPosBool(self, bool):
		self.isTeachedPos = bool
	def setRequestPosBool(self,bool):
		self.requestPos=bool
	def setExecuteSequenceBool(self,bool):
		self.executeSequenceBool=bool
	def setSequenceIndex(self,index):
		self.sequenceIndex = index
