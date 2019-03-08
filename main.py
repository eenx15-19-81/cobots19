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
from std_msgs.msg import String, Bool
from geometry_msgs.msg import WrenchStamped, Wrench
#from cobots19.msg import Buttons, LED

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
		rospy.Subscriber("/joint_states",JointState,self.robotCallback)
		rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
		rospy.Subscriber("/buttons", Buttons, self.buttons_callback)
		rospy.Subscriber("/Robotiq2FGripperRobotInput",outputMsg.Robotiq2FGripper_robot_output,self.gripperCallback)
		time.sleep(1)

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
		time.sleep(1)

		## Initialization complete, ready for the work in workspace to be done.
		self.workspace()
	
	## Our main workspace for the programming itself. This is where you put stuff to be tried.
	# To your use you will have the subclasses folder where most of the functions are.
	def workspace(self):
		self.modeSelBool = True
		print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:5 to exit "
		while not rospy.is_shutdown():
			if self.modeSelBool:
				if self.m.freedrivebool:
					print "Entered freedrive mode"
					self.modSelBool = False
					self.m.freedrive()
					self.modeSelBool = True
				elif self.m.teachModeBool:
					print "Entered teach mode"
					self.modeSelBool = False
					self.m.teachmode()
					print "Learning your moves..."
					time.sleep(1)
					self.m.setMove2TeachedPosBool(True)
					self.m.move2TeachedPos()
				elif self.m.move2pobool:
					print "Entered predefined move mode"
					self.modeSelBool = False
					self.m.move2pos()
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
		self.currentGripperrPR=data.rPR

	# Callback from the buttons on the Raspberry that updates the booleans that describes what mode that we want to enter.
	# Input: bool (Button msg)
	def buttonsCallback(self,data):
		print "buttoncallback"
		if self.modeSelBool:
			if data.button1:
				print "Button1 pressed"
				self.m.freedrivebool=True
			elif data.button2:
				print "Button2 pressed"
				self.m.teachModeBool=True
			elif data.button3:
				print "Button3 pressed"
				self.m.move2pobool=True
			elif data.button5:
				rospy.signalShutdown('Shutting down')
				print "Shutting down..."
		else:
			if self.m.freedrivebool:
				self.m.freedriveButton(data)
			elif self.m.teachModeBool:
				self.m.teachModeButton(data)
			elif self.m.move2TeachedPosBool:
				self.m.moveTeachModeButton(data)
			elif self.m.move2pobool:
				self.m.preDefinedButton(data)

	#TODO comment
	# Input: True, False
	def threadWait(self,bool):
		while True:
			bo=raw_input("Type 'exit' to quit: ")
			if bo =='exit':
				if self.m.freedrivebool: 
					self.m.setfreedrivebool(bool)	
				elif self.m.move2pobool:
					self.m.setMove2posbool(bool)		
				elif self.m.teachmode:
					self.m.setTeachModeBool(bool)
				elif self.m.move2TeachedPosBool:
					self.m.setMove2TeachedPosBool(bool)
				else: 
					self.m.setIsTeachedPosBool(bool)
				thread.exit()

	# When modeSelBool=True you are in mode selection. This method sets that variable based on your input argument.
	# Input: True, False
	def setModeSelBool(self,bool):
		self.modeSelBool=bool

try:
	main()
except rospy.ROSInterruptException:
	pass
