#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import thread
import math
import tf

from sensor_msgs.msg import JointState

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import WrenchStamped, Wrench

from subclasses import robot 
from subclasses import gripper
from subclasses import optoForce
import mode

class main():
	joint_home=[0,-math.pi/2,0,-math.pi/2, 0, 0]
	joint_pose2=[0.995, -1, -2.013, -2.652, -0.140, -0.532]
	def __init__(self):
		## Initializing instances of subclasses
		self.r=robot.robot()
		self.g=gripper.gripper()
		
		
		
		## Initializing node and setting up topics
		rospy.init_node('main',anonymous=True)
		self.rate=rospy.Rate(125)
		self.urPublisher=rospy.Publisher('/ur_driver/URScript',String,queue_size=10)
		self.gripperPub = rospy.Publisher('/Robotiq2FGripperRobotOutput',outputMsg.Robotiq2FGripper_robot_output , queue_size=10)
		self.optoZeroPub = rospy.Publisher('/ethdaq_zero',Bool,queue_size=1)
		rospy.Subscriber("/wrench",WrenchStamped,self.wrenchCallback)
		rospy.Subscriber("/joint_states",JointState,self.robotCallback)
		rospy.Subscriber("/ethdaq_data", WrenchStamped, self.wrenchSensorCallback)
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
		time.sleep(2)

		## Initialization complete, ready for the work in workspace to be done.
		self.workspace()
	
	## Our main workspace for the programming itself. This is where you put stuff to be tried.
	# To your use you will have the subclasses folder where most of the functions are.
	def workspace(self):
		thread.start_new_thread(self.threadWait,(False,))
		self.m.freedrive()
		# Demo move
		# while not rospy.is_shutdown():
		# 	self.robotTalk(self.r.move(self.joint_home))
		# 	self.r.waitForMove(0.001, self.joint_home)
		# 	self.gripperTalk(self.g.open())
		# 	time.sleep(1)
		# 	self.gripperTalk(self.g.close())
		# 	time.sleep(1)
		# 	self.robotTalk(self.r.move(self.joint_pose2))
		# 	self.r.waitForMove(0.001, self.joint_pose2)

		# First freedrive
		#while not rospy.is_shutdown():
		#	self.robotTalk(self.r.move(self.joint_pose2))
		#	self.r.waitForMove(0.001, self.joint_pose2)
		#	time.sleep(1)
		#	self.optoZeroPub.publish(True)
		#	time.sleep(2)
		#	print("ready to move")
		#	while not rospy.is_shutdown():
		#		self.robotTalk(self.o.getSpeedl())
		#		self.rate.sleep() 
		#	self.robotTalk("stopl(1) \n")
		#self.m.move()
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
	# Callback from the force in the joints in the robot
	def wrenchCallback(self,data):
		self.o.setRobotForce([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
		self.o.setRobotTorque([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

	# Callback from the opto sensor with forces and torque and updates the optoForce with current force and torque
	def wrenchSensorCallback(self,data):
		self.o.averageForceMatrix[0].pop(0)
		self.o.averageForceMatrix[1].pop(0)
		self.o.averageForceMatrix[2].pop(0)

		self.o.averageForceMatrix[0].append(data.wrench.force.x)
		self.o.averageForceMatrix[1].append(data.wrench.force.y)
		self.o.averageForceMatrix[2].append(data.wrench.force.z)

		self.o.setCurrentForce([self.o.averageOfList(self.o.averageForceMatrix[0]),self.o.averageOfList(self.o.averageForceMatrix[1]), self.o.averageOfList(self.o.averageForceMatrix[2])])
		#self.o.setCurrentForce([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
		self.o.setCurrentTorque([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
	""" 	with open("forceSensorData.txt", "a+") as filehandle:  
			for listitem in self.o.curForce:
				filehandle.write('%s\n' % listitem) """

	def threadWait(self,bool):
		bo=raw_input("close to close: ")
		self.m.setfreedrivebool(bool)
		print "exit"
		thread.exit()


try:
	main()
except rospy.ROSInterruptException:
	pass
'''
def workspace(self):
		isModeSelected = False
		modeName = ''
		# Demo move
		# while not rospy.is_shutdown() or not isModeSelected:
		# 	self.robotTalk(self.r.move(self.joint_home))
		# 	self.r.waitForMove(0.001, self.joint_home)
		# 	self.gripperTalk(self.g.open())
		# 	time.sleep(1)
		# 	self.gripperTalk(self.g.close())
		# 	time.sleep(1)
		# 	self.robotTalk(self.r.move(self.joint_pose2))
		# 	self.r.waitForMove(0.001, self.joint_pose2)
		
		
		#	Mode selection: Have the ability to select a new mode while 
		#					the robot is still in action without disturbing	the code 
		#					which is handling the robot's current action (eg. demoMove)isModeSelect = False
		#				NOT DONE YET!
			modeName = raw_input('Do you wish to continue? [Y/N]')
			if modeName == y or modeName == Y
				continue
			elif modeName == N or modeName == n 
				isModeSelected = True
				break

		modeSelection = mode_selection_with_timer()
		if modeSelection == 'mm' 
			self.m.move2pos() 	
		else 
			self.m.moveFree()
		

	def mode_selection_with_timer(timeout = 5.0):	#NOT DONE YET! 
		timer = threading.Timer(timeout, thread.interupt_main)
		str = None
		try:
			timer.start()
			str = raw_input('Free-drive (fd) or moveMove (mm)?')
		except KeyboardInterrupt:
				pass
		timer.cancel()
		return str
		'''