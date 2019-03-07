#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import thread
import math

from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import WrenchStamped, Wrench
from cobots19.msg import Buttons, LED

from subclasses import robot 
from subclasses import gripper
from subclasses import optoForce


import mode
import tf

class main():
	joint_home=[0,-math.pi/2,0,-math.pi/2, 0, 0]
	joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	workspaceBool = True
	mode_sel_bool = False

	def __init__(self):
		## Initializing instances of subclasses
		self.r=robot.robot()
		self.g=gripper.gripper()

		
		## Initializing node and setting up topics
		rospy.init_node('main',anonymous=True)
		self.rate=rospy.Rate(125)
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		self.led_publisher = rospy.Publisher('/led', LED, queue_size = 10)
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
		self.optoZeroPub = rospy.Publisher('/ethdaq_zero',Bool,queue_size=1)
		rospy.Subscriber("/joint_states",JointState,self.robotCallback)
		rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
		rospy.Subscriber("/buttons", Buttons, self.buttons_callback)
		self.o=optoForce.optoForce(tf,rospy) ## test
		self.m=mode.mode(self.r,self.g,self.o,self)
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
		self.mode_sel_bool = True
		print "Button:1 for Freedrive, Button:2 for Teaching, Button:3 for Predefinied Actions, Button:5 to exit "
		rospy.spin()

	# Publishes messages to the gripper
	def gripperTalk(self, msg):
		self.gripperPub.publish(msg)
		time.sleep(1)
	# Publishes messages to the robot
	def robotTalk(self,msg):
		self.urPublisher.publish(msg)
	# Callback from the URSubscriber updating jointstates in robot subclass with current position
	def robotCallback(self,data):
		self.r.setCurrentPosition(data.position)
	# Callback from the opto sensor with forces and torque and updates the optoForce with current force and torque
	def wrenchSensorCallback(self,data):
		self.o.setCurrentForce([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
		self.o.setCurrentTorque([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
	# ...
	def buttons_callback(self,data):
		if self.mode_sel_bool:
			if data.button1:
				self.m.freedrivebool=True
				self.mode_sel_bool = False
				self.m.freedrive(False)
				self.mode_sel_bool = True
			elif data.button2:
				self.m.teachModeBool=True
				self.mode_sel_bool = False
				self.m.teachmode()
				print "Learing your moves..."
				time.sleep(2)
				self.m.set_move2TeachedPosBool(True)
				self.m.move2TeachedPos()
				self.mode_sel_bool = True
			elif data.button3:
				self.m.move2pobool=True
				self.mode_sel_bool = False
				self.m.move2pos()
				self.mode_sel_bool = True
			elif data.button5:
				rospy.signal_shutdown('Shutting down')


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
					self.m.set_move2TeachedPosBool(bool)
				else: 
					self.m.set_isTeachedPos_Bool(bool)
				thread.exit()


try:
	main()
except rospy.ROSInterruptException:
	pass
