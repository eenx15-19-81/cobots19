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
	alignOrigo=[0,0,0,0,0,0]
	alignAngle=0
	
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

	tableLength=1
	tableWidth=1

	# Initializing and creating instances of robot, gripper, optoforce and main
	def __init__(self,r,g,o,main):
		self.r = r
		self.g = g
		self.o = o
		self.main = main

	#######################
	####### Modes #########
	#######################

	# Starts up the freedrive mode
	def freedrive(self):
		while self.freedriveBool:
			time.sleep(1)
			print("WARNING, robot will move.\n")
			time.sleep(1)
			self.main.moveRobotPosition(self.o.calibration(),0.005,'xyz')
			time.sleep(2)
			self.main.optoZeroPub.publish(True)
			time.sleep(2)
			print(self.freedriveBool)
			while self.freedriveBool:
				self.main.customRobotMessage(self.o.getSpeedl())
				self.main.rate.sleep() 
			self.main.stopRobot()
	
	# Starts upp the teaching mode
	def teachmode(self):
		self.storedList=[]
		time.sleep(1)
		self.main.optoZeroPub.publish(True)
		time.sleep(2)
		print "Ready"
		#f=open('teachmode.txt','a+')
		thread.start_new_thread(self.storePos1,('teach',))
		while self.teachModeBool:
			#f.write(str(self.storeCurrentPosition())+'\n')
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
			self.main.customRobotMessage(self.o.getSpeedl())
			self.main.rate.sleep() 
		self.main.stopRobot()
		#f.close()
		var = raw_input("Save or exit?")
		if var=="save":
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
							print self.storedList[z]
							print self.alignOrigo[0]+abs(self.tableWidth*math.cos(self.alignAngle))
							print self.alignOrigo[0]
							print self.alignOrigo[1]+abs(self.tableLength*math.cos(self.alignAngle))
							print self.alignOrigo[1]
							if self.storedList[z][0] < self.alignOrigo[0]+abs(self.tableWidth*math.cos(self.alignAngle)):
								if self.storedList[z][0] > self.alignOrigo[0]: 
									if self.storedList[z][1] < self.alignOrigo[1]+abs(self.tableLength*math.cos(self.alignAngle)):
										if self.storedList[z][1] > self.alignOrigo[1]:
											self.switchList.append(z)
			# Make Method for easy piping
			storedSequence=open("storedSequence.txt", "a+")
			storedSequence.write(str(self.storedList))
			storedSequence.write("|")
			storedSequence.write(str(self.switchList))
			storedSequence.write("|")
			storedSequence.write(str(self.alignOrigo))
			storedSequence.write("|")
			storedSequence.write(str(self.alignAngle))
			storedSequence.write("\n")
			storedSequence.close()
			print "Done learning"

		print "Exiting to mode selector"

	# Chooses and executes the desired sequenced based on the input.
	# Input: int (Desired sequence number)
	def chooseAndExecuteSeq(self):
		while self.executeSequenceBool:
			if not self.sequenceIndex == None:
				thread.start_new_thread(self.storePos1,('ex',))
				[sequence, self.switchList,self.alignOrigo,self.alignAngle] = self.getSequence(self.sequenceIndex)
				mainSequence=list(sequence)
				while self.executeSequenceBool and not self.sequenceIndex == None:
					for x in range (0,len(sequence)):
						if type(sequence[x]) is list:
							self.main.moveRobotPosition(sequence[x],0.005,'xyz',3)
						elif type(sequence[x]) is str:
							if sequence[x] == "Open":
								self.main.gripperTalk(self.g.open())
							elif sequence[x] == "Close":
								self.main.gripperTalk(self.g.close())
							elif sequence[x] == "Align":
								self.main.moveRobotPosition(sequence[x+1],0.005,'xyz',3)
								time.sleep(0.5)
								[curOrigo,curAngle]=self.align()
								curAngle=curAngle-self.alignAngle
								if not self.switchList == None:
									for y in range(0,len(self.switchList)):
										xx=mainSequence[self.switchList[y]][0]-self.alignOrigo[0]
										yy=mainSequence[self.switchList[y]][1]-self.alignOrigo[1]
										xprim = xx*math.cos(curAngle)-yy*math.sin(curAngle)+curOrigo[0]
										yprim = xx*math.sin(curAngle)+yy*math.cos(curAngle)+curOrigo[1]
										print xprim
										print yprim
										sequence[self.switchList[y]][0] = xprim
										sequence[self.switchList[y]][1] = yprim
									x+=1
							else:
								print "There is a fault in the sequence, it contains a string that is not 'Open' or 'Close'"		
	def storePos1(self,a):
		d=open(str(a)+'.txt','a+')
		while self.executeSequenceBool or self.teachModeBool:
			d.write(str(self.storeCurrentPosition())+'\n')
			time.sleep(0.05)
		d.close()
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
		storedSequence = open("storedSequence.txt","r+")
		lines = storedSequence.read().splitlines()
		program = lines[int(line)].split('|')
		storedSequence.close()
		return [ast.literal_eval(program[0]), ast.literal_eval(program[1]), ast.literal_eval(program[2]), float(program[3])]


	##################################
	####### Button callbacks #########
	##################################

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
	def executeSequenceModeButton(self,data):
		if data.button5:
			self.executeSequenceBool=False
			self.main.setModeSelBool(True)

	# Defines what the buttons will do while in predefined mode by sending in a msg as an argument. Button5 exits the mode. 
	# Input: msg (Button)
	def preDefinedButton(self,data):
		if data.button5:
			self.move2PredefBool=False
			self.main.setModeSelBool(True)
			print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:4 for saved programs, Button:5 to exit "	

	######################
	####### Misc #########
	######################

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
		storedSequence = open("storedSequence.txt","r+")
		lines = storedSequence.read().splitlines()
		program = lines[int(line)].split('|')
		storedSequence.close()
		return [ast.literal_eval(program[0]), ast.literal_eval(program[1]), ast.literal_eval(program[2]), float(program[3])]

	def align(self):
		startPos = self.r.getCurrentPosition()
		self.moveInDirection(np.array([-1,0,0]),2,2)
		wallPos1 = self.r.getCurrentPosition()
		self.main.moveRobotPosition(startPos,0.005,'xyz',3)
		time.sleep(0.5)
		startPosCopy=[startPos[0],startPos[1]-0.1,startPos[2],startPos[3],startPos[4],startPos[5]]
		self.main.moveRobotPosition(startPosCopy,0.005,'xyz',3)
		time.sleep(0.5)
		self.moveInDirection(np.array([-1,0,0]),2,2)
		wallPos2 = self.r.getCurrentPosition()
		x = wallPos2[0]-wallPos1[0]
		y = wallPos2[1]-wallPos1[1]
		if wallPos2[0]<wallPos1[0]:
			alpha = math.atan(y/x)-pi/2
		else:
			alpha = pi/2+math.atan(y/x)
		movement = [wallPos2[0]+0.01,wallPos2[1],wallPos2[2],wallPos2[3],wallPos2[4],wallPos2[5]]
		self.main.moveRobotPosition(movement,0.005,'xyz',3)
		time.sleep(2)
		self.moveInDirection([-np.cos(pi/2+alpha),-np.sin(pi/2+alpha),0],2,2)
		origo = self.r.currentPosition
		return [origo,alpha]

	#Direction as [x,y,z] where x,y,z = 0,-1 or 1
	def moveInDirection(self, direction, forceX, forceY):
		while self.checkOpto(forceX,forceY):
			self.main.moveRobotDirection(0.05,direction)
			self.main.rate.sleep() 
		self.main.stopRobot()

	def checkOpto(self,forceX, forceY):
		curForce = self.o.getCurForce()
		if curForce[0] > forceX or curForce[1] > forceY: 
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
