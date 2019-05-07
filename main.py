#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import thread
import math
import mode
import tf

from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import WrenchStamped, Wrench
from cobots19.msg import Buttons, LED, State, Command

from subclasses import robot 
from subclasses import gripper
from subclasses import optoForce



class main():
	jointHome=[0,-math.pi/2,0,-math.pi/2, 0, 0]				# Hardcoded position for the predefined mode.
	jointPose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]	# Hardcoded position for the predefined mode.

	# Initialization of our main booleans and the initial pressure of the gripper.
	workspaceBool = True
	modeSelBool = False
	currentGripperrPR=0

	#Local variables for auto mode
	commandName = ""
	commandRun = False
	commandProduct = ""
	state = "init"
	#init = True
	#executing = False
	#finished = False

	def __init__(self):
		## Initializing instances of subclasses
		self.r=robot.robot()
		self.g=gripper.gripper()
		
		## Initializing node and setting up topics
		rospy.init_node('main',anonymous=True)
		self.o=optoForce.optoForce(tf,rospy) ## test
		self.m=mode.mode(self.r,self.g,self.o,self)
		self.rate=rospy.Rate(125)
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		self.ledPublisher = rospy.Publisher('/led', LED, queue_size = 10)
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
		self.optoZeroPub = rospy.Publisher('/ethdaq_zero',Bool,queue_size=1)
		self.masterPub = rospy.Publisher('/state',State) #Unsure of topic name
		rospy.Subscriber("/joint_states",JointState,self.robotCallback)
		rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
		rospy.Subscriber("/buttons", Buttons, self.buttonsCallback)
		rospy.Subscriber("/Robotiq2FGripperRobotInput",inputMsg.Robotiq2FGripper_robot_input,self.gripperCallback)
		rospy.Subscriber("/command", Command) #Unsure of topic name
		time.sleep(1)

		self.ledPublisher.publish(led1=True,led2=True,led3=True)

		## Activating gripper
		# Sending first a reset to the gripper so if it have been wrongly activated before it will be a fresh start when our init runs.
		# Sleep 0.1s and then start the activating sequence.
		msgReset = outputMsg.Robotiq2FGripper_robot_output()
		msgReset.rACT = 0
		self.gripperPub.publish(msgReset)
		time.sleep(0.3)
		msgActivate = outputMsg.Robotiq2FGripper_robot_output()
		msgActivate.rACT=1
		msgActivate.rGTO=1
		msgActivate.rSP=255
		msgActivate.rFR=10
		self.gripperPub.publish(msgActivate)
		time.sleep(2)

		## Initialization complete, ready for the work in workspace to be done.
		self.workspace()
	
	## Our main workspace for the programming itself. This is where you put stuff to be tried.
	# To your use you will have the subclasses folder where most of the functions are.
	def workspace(self):
		self.modeSelBool = True
		print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:4 for saved programs, Button:5 to exit "		
		while not rospy.is_shutdown():
			if self.modeSelBool:
				if self.m.freedriveBool:
					print "Entered freedrive mode"
					self.modeSelBool = False
					self.m.freedrive()
					self.modeSelBool = True
				elif self.m.teachModeBool:
					print "Entered teach mode"
					self.modeSelBool = False
					self.m.teachmode()
					self.modeSelBool = True
				elif self.m.move2PredefBool:
					print "Entered predefined move mode"
					self.modeSelBool = False
					self.m.move2Predef()
					self.modeSelBool = True
				elif self.m.executeSequenceBool:
					print "Entered choose-and-execute-sequence mode"
					print "Enter program on button 1,2,3,4 or exit on button 5"
					self.modeSelBool = False
					self.m.chooseAndExecuteSeq()
					self.modeSelBool = True
				elif self.m.autoBool:
					print "Entering Auto mode"
					self.modeSelBool = False
					self.runAuto()
					self.modeSelBool = True

		rospy.spin()

	# Publishes messages to the gripper.
	# Input: msg (Robotiq msg)
	def gripperTalk(self, msg):
		self.gripperPub.publish(msg)
		time.sleep(1)

	# Publishes messages to the robot.
	# Input: msg (UR10 msg)
	def robotTalk(self,msg):
		self.urPublisher.publish(msg)

	# Callback from the URSubscriber updating jointstates in robot subclass with current position.
	# Input: msg (Jointstate)
	def robotCallback(self,data):
		self.r.setCurrentPosition(data.position)

	# Callback from the opto sensor with forces and torque and updates the optoForce with current force and torque.
	# Input: msg (WrenchStamped)
	def wrenchSensorCallback(self,data):
		self.o.setCurrentForce([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
		self.o.setCurrentTorque([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

	# Callback from the gripper with the pressure that it is applying and updates the gripper with the current pressure.
	# Input: int (Robotiq msg)
	def gripperCallback(self,data):
		self.currentGrippergPR=data.gPR

	def runAuto(self):
		while self.m.autoBool and not rospy.is_shutdown():
			if self.state == "init":
				self.masterPub.publish(self.state, self.commandName, message="Waiting for run")
				if self.commandRun:
					self.state = "executing"
					self.masterPub.publish(self.state, self.commandName, "running")
			elif self.state == "executing":
				if commandName == "assemble": #Phittad string
					self.m.chooseAndExecuteSeq(0)
					self.state = "finished"
					self.masterPub.publish(self.state, self.commandName, "finished")
			elif self.state == "finished":
				if not self.commandRun:
					self.state = init



			
		# Check what state we are in
		# Init:
		# Listen for command and update the /state topic so that it matches the /commnd topic.
		# Did we recive run True? If so, set state to Executing.
		# Executing:
		# Run the specified command.
		# When done, change to finished state.
		# Finished:
		# publish 
		# Wait for not run.
		# set state to init.


	
	def masterCallback(self,data):
		self.commandName = data.command
		self.comandRun = data.run
		self.commandProduct = data.product_name
		# Update local variables


	# Callback from the buttons on the Raspberry that updates the booleans that describes what mode that we want to enter.
	# Input: bool (Button msg)
	def buttonsCallback(self,data):
		if self.modeSelBool:
			if data.button1:
				print "Button1 pressed"
				self.ledPublisher.publish(led1=True,led2=False,led3=False)
				self.m.freedriveBool=True
			elif data.button2:
				print "Button2 pressed"
				self.ledPublisher.publish(led1=False,led2=True,led3=False)
				self.m.teachModeBool=True
			elif data.button3:
				print "Button3 pressed"
				self.ledPublisher.publish(led1=False,led2=False,led3=True)
				self.m.move2PredefBool=True
			elif data.button4:
				print "Button4 pressed"
				self.m.executeSequenceBool=True
			elif data.button5:
				print "Shutting down..."
				self.ledPublisher.publish(led1=False,led2=False,led3=False)
				rospy.signal_shutdown('Shutting down')
		else:
			if self.m.freedriveBool:
				self.m.freedriveButton(data)
			elif self.m.teachModeBool:
				self.m.teachModeButton(data)
			elif self.m.executeSequenceBool:
				self.m.chooseAndExecuteSeqButton(data)
			elif self.m.move2PredefBool:
				self.m.preDefinedButton(data)
				
	# When modeSelBool=True you are in mode selection. This method sets that variable based on your input argument.
	# Input: True, False
	def setModeSelBool(self,bool):
		self.modeSelBool=bool

try:
	main()
except rospy.ROSInterruptException:
	pass
