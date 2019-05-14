#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import thread
import math
import mode
import tf
import numpy as np

from sensor_msgs.msg import JointState

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped
from cobots19.msg import Buttons, LED

from subclasses import robot 
from subclasses import gripper
from subclasses import optoForce



class main():

	# Initialization of our main booleans and the initial pressure of the gripper.
	workspaceBool = True
	modeSelBool = False
	currentGripperrPR=0

	def __init__(self):
		## Initializing instances of subclasses
		self.r=robot.robot()
		self.g=gripper.gripper()
		
		rospy.init_node('main',anonymous=True)

		## Initializing subclasses with their init statements
		self.o = optoForce.optoForce(tf,rospy)
		self.m = mode.mode(self.r,self.g,self.o,self)
		self.rate = rospy.Rate(125)

		## Initializing all the publishers
		self.urPublisher = rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		self.ledPublisher = rospy.Publisher('/led', LED, queue_size = 10)
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
		self.optoZeroPub = rospy.Publisher('/ethdaq_zero',Bool,queue_size=1)

		## Initializing The ubscribers and their callback functions
		rospy.Subscriber("/buttons",Buttons,self.buttonsCallback)
		rospy.Subscriber("/tf",TransformStamped,self.vectorCallback)
		#rospy.Subscriber("/wrench",WrenchStamped,self.wrenchCallback)
		#rospy.Subscriber("/joint_states",JointState,self.robotCallback)
		rospy.Subscriber("/ethdaq_data",WrenchStamped,self.wrenchSensorCallback)
		rospy.Subscriber("/Robotiq2FGripperRobotInput",inputMsg.Robotiq2FGripper_robot_input,self.gripperCallback)
		time.sleep(1)

		## Activating gripper
		self.g.activateGripper(self)

		## Initialization complete, ready for the work in workspace to be done.
		self.workspace()

	######################################
	############ Main loop ###############
	######################################
	
	## Our main workspace for the programming itself. This is where you put stuff to be tried.
	# To your use you will have the subclasses folder where most of the functions are.
	def workspace(self):
		self.modeSelBool = True

		## Setting all the lights ON indicating the operator is in modeselection
		self.ledPublisher.publish(led1=True,led2=True,led3=True)

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
				#elif self.m.move2PredefBool:
					#print "Entered predefined move mode"
					#self.modeSelBool = False
					#self.m.move2Predef()
					#self.modeSelBool = True	
				elif self.m.executeSequenceBool:
					print "Entered choose-and-execute-sequence mode"
					print "Enter program on button 1,2,3,4 or exit on button 5"
					self.modeSelBool = False
					self.m.chooseAndExecuteSeq()
					self.modeSelBool = True
		rospy.spin()

	######################################
	############ Publishers ##############
	######################################

	# Publishes messages to the gripper.
	# Input: msg (Robotiq msg), standard messages found in the gripper class 
	# Sleeps for 1 second to make the code wait for the gripper to have fully gripped.
	def gripperTalk(self, msg):
		self.gripperPub.publish(msg)
		time.sleep(1)

	# Publishes messages that moves the robot to a certain position in linear toolspace to the robot. 
	# Input: pos, 6 positional float[] with the format of [x, y, z, rot_x, rot_y, rot_z]
	# Input: margin, the margin of error in the position
	# Input: numberOfIndices that is used with the margin, either 3 or 6
	# Input: acceleration in m/s 
	# Input: velocity, maximum velocity in m/s
	# Input: SHOULD IT BE TIME HERE??!??!?!??!
	# Input: radius of the movement between the points (correct) !?!?!?!??!
	def moveRobotPosition(self, pos, margin, type, numberOfIndices = 6, acceleration = 0.1, velocity = 0.1, time = 0, radius = 0):
		self.urPublisher.publish(self.r.getMoveMessage(pos,type, acceleration, velocity, time, radius))
		self.r.waitForMove(margin,pos,type,numberOfIndices)

	# Stops the robots current movement
	def stopRobot(self):
		self.customRobotMessage("stopl(1) \n")

	# Publishes messages to the robot.
	# Input: msg (UR10 msg)
	def customRobotMessage(self,msg):
		self.urPublisher.publish(msg)

	# Publishes a message to move the robot in a certain direction
	# Input: velocity as the maximum speed in each direction.
	# Input: direction, the direction that the robot should move in as a list of 3 variables between 0 and 1.
	def moveRobotDirection(self, velocity, direction, acceleration = 0.5, time = 0.05):
		velocityVector=np.concatenate((np.multiply(direction,velocity), np.array([0.0,0.0,0.0])))
		self.urPublisher.publish(self.r.getSpeedlCommand(velocityVector, acceleration, time))

	######################################
	####### Subscriber callbacks #########
	######################################

	# Callback from the URSubscriber updating jointstates in robot subclass with current position
	#def robotCallback(self,data):
		#self.r.setCurrentPosition(data.position)
	# Callback from the force in the joints in the robot
	"""def wrenchCallback(self,data):
		self.o.setRobotForce([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
		self.o.setRobotTorque([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])"""

	# Callback from the opto sensor with forces and torque and updates the optoForce with current force and torque
	def wrenchSensorCallback(self,data):
		self.o.averageForceMatrix[0].pop(0)
		self.o.averageForceMatrix[1].pop(0)
		self.o.averageForceMatrix[2].pop(0) 
		self.o.averageForceMatrix[0].append(data.wrench.force.x)
		self.o.averageForceMatrix[1].append(data.wrench.force.y)
		self.o.averageForceMatrix[2].append(data.wrench.force.z)
		self.o.setCurrentForce([self.o.averageOfList(self.o.averageForceMatrix[0]),self.o.averageOfList(self.o.averageForceMatrix[1]), self.o.averageOfList(self.o.averageForceMatrix[2])])
		self.o.setCurrentTorque([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
		self.o.setRawForce([data.wrench.force.x,data.wrench.force.y,data.wrench.force.z])	
		"""with open("forceSensorData.txt", "a+") as filehandle:  
			for listitem in self.o.rawForce:
				filehandle.write('%s\n' % listitem)

        	with open("compensatedData.txt", "a+") as filehandle:  
			for listitem in self.o.curForce:
				filehandle.write('%s\n' % listitem)"""
			

	# Callback from the gripper with the pressure that it is applying and updates the gripper with the current pressure.
	# Input: int (Robotiq msg)
	def gripperCallback(self,data):
		self.currentGrippergPR=data.gPR

	def vectorCallback(self,data):
		if data.transforms[0].child_frame_id == "tool0_controller":
			x = data.transforms[0].transform.translation.x
			y = data.transforms[0].transform.translation.y
			z = data.transforms[0].transform.translation.z
		#	print str(x) + " and " +str(y)
			rx = data.transforms[0].transform.rotation.x
			ry = data.transforms[0].transform.rotation.y
			rz = data.transforms[0].transform.rotation.z
			rw = data.transforms[0].transform.rotation.w
			self.r.setCurrentPosition(self.quat_to_rot(x,y,z,rx,ry,rz,rw))

	def quat_to_rot(self, x, y, z, qx, qy, qz, qw):
		rpy = [0, 0, 0, 0, 0, 0]
		rpy = self.quat_to_rpy(x, y, z, qx, qy, qz, qw)
		roll = rpy[3]
		pitch = rpy[4]
		yaw = rpy[5]
		rot = self.rpy_to_rot(x, y, z, roll, pitch, yaw)
		return rot

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
				self.ledPublisher.publish(led1=True,led2=True,led3=True)
			elif self.m.teachModeBool:
				self.m.teachModeButton(data)
				self.ledPublisher.publish(led1=True,led2=True,led3=True)
			elif self.m.executeSequenceBool:
				self.m.chooseAndExecuteSeqButton(data)
			elif self.m.move2PredefBool:
				self.m.preDefinedButton(data)
				self.ledPublisher.publish(led1=True,led2=True,led3=True)
				
	# When modeSelBool=True you are in mode selection. This method sets that variable based on your input argument.
	# Input: True, False
	def setModeSelBool(self,bool):
		self.modeSelBool=bool

	def quat_to_rpy(self, x, y, z, qx, qy, qz, qw):
		quaternions = (qx, qy, qz, qw)
		euler = tf.transformations.euler_from_quaternion(quaternions)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		return [x, y, z, roll, pitch, yaw]


	def rpy_to_quat(self, x, y, z, roll, pitch, yaw):
		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		qx = quaternion[0]
		qy = quaternion[1]
		qz = quaternion[2]
		qw = quaternion[3]
		return [x, y, z, qx, qy, qz, qw]


	def rpy_to_rot(self, x, y, z, roll, pitch, yaw):
		yawMatrix = np.matrix([
		[math.cos(yaw), -math.sin(yaw), 0],
		[math.sin(yaw), math.cos(yaw), 0],
		[0, 0, 1]
		])
		pitchMatrix = np.matrix([
		[math.cos(pitch), 0, -math.sin(pitch)],
		[0, 1, 0],
		[math.sin(pitch), 0, math.cos(pitch)]
		])

		rollMatrix = np.matrix([
		[1, 0, 0],
		[0, math.cos(roll), -math.sin(roll)],
		[0, math.sin(roll), math.cos(roll)]
		])

		R = yawMatrix * pitchMatrix * rollMatrix
		theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
		multi = 1 / (2 * math.sin(theta))
		rx = multi * (R[2, 1] - R[1, 2]) * theta
		ry = multi * (R[0, 2] - R[2, 0]) * theta
		rz = multi * (R[1, 0] - R[0, 1]) * theta
		return [x, y, z, rx, ry, rz]

try:
	main()
except rospy.ROSInterruptException:
	pass
