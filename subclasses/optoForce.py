#! /usr/bin/python
import numpy as np
import math
import time


from geometry_msgs.msg import TwistStamped, WrenchStamped

class optoForce():
    curForce=[]
    curTorque=[]

    # Increase if the robot "wanders" when in forcecontrol
    deadbandForce=0.03
    deadbandTorque=0.2
    deadbandVelocity=0.2
    def __init__(self,tf,rospy):
        self.forceError=[0,0,0]
        self.forceReference=[0,0,0]
        self.tf = tf
        self.rospy = rospy
        print("Initialized optoForce class")
        self.listener = tf.TransformListener()
        self.integralPart=[0.0,0.0,0.0]
        self.derivativePart=[0.0,0.0,0.0]

    def setCurrentForce(self, curForce):
        self.curForce=curForce

    def setCurrentTorque(self,curTorque):
        self.curTorque=curTorque

    ## Get current transform matrix for frame1 to frame2 conversion from tf.
    def transformMatrix(self, frame1, frame2):
        self.listener.waitForTransform( '/'+frame1, '/'+frame2, self.rospy.Time(),self.rospy.Duration(.1))
        (trans,rot) = self.listener.lookupTransform('/'+frame1, '/'+frame2, self.rospy.Time(0))
        return self.listener.fromTranslationRotation(trans, rot)

    ## Convert a desired-velocity vector from toolframe to baseframe using tf.
    def convertFrame(self, velocity):
        rotationMatrix = self.transformMatrix('base','tool0_controller')
        linearVelocity = np.matmul(rotationMatrix[0:3,0:3], velocity[0:3]) 
        angularVelocity = np.matmul(rotationMatrix[0:3,0:3], velocity[3:6])
        velocity = np.concatenate((linearVelocity, angularVelocity)) 
        return velocity

    ## Returns a speedl command based on forces read by sensor. This can later be sent to the robots URScript channel. See URScript docs for more information.
    # rotAcceleration: a separate acceleration value used only for orientation changes (for rx, ry and rz). Unit is rad/(s*s). 
    # Generally higher than the normal acceleration for better "feel" and less delay.
    # time: maximum amount of time the robot will spend running the command. 
    def getSpeedl(self, acceleration = 0.9, rotAcceleration=1.2, time=0.05):
        velocity=self.forceControl()
        command = "speedl(" + np.array2string(velocity, precision= 5, separator=',') +", "+ \
        str(acceleration) + ", " + str(time) + ", " + str(rotAcceleration) +")" 
       # self.rospy.loginfo(command)
        return command

    ## Returns the desired tool velocites in vector form (x, y, z, rx, ry, rz) based on force and torque readings.
    # kp_force and kp_torque can be increased for higher sensitivity and lowered for less sensitivity.
    def forceControl(self, kp_force=0.02, kp_torque=0.4):
        force = np.array(self.curForce)
        torque = np.array(self.curTorque)
        #TODO selction_vector
        force = kp_force * force
        torque = kp_torque * torque
        velocity = np.concatenate([force, torque])
        rotationMatrix = self.transformMatrix('tool0_controller','base')

        gravitationForce = np.matmul(rotationMatrix[0:3,0:3], [0,0,-0.9*9.82])
        length = 1/(9.82 * 1.8)
        Tx = kp_torque * length * gravitationForce[0]
        Ty = kp_torque * length * gravitationForce[1]

        velocity=np.subtract(velocity,[0,0,-kp_force*0.9*9.82,Tx,Ty,0])
        velocity = self.convertFrame(velocity)
        velocity=np.subtract(velocity,[0,0,-kp_force*0.9*9.82,0,0,0])
        #print velocity 
        velocity=self.checkInDeadband(velocity)
        return velocity
    def checkInDeadband(self,velocity):
        for x in range(3):
            if abs(velocity[x]) < self.deadbandForce:
                velocity[x]=0
            else:
                velocity[x]=velocity[x]-np.sign(velocity[x])*self.deadbandForce
        for x in range(3):
            if abs(velocity[x+3]) < self.deadbandTorque:
                velocity[x+3]=0
            else:
                velocity[x+3]=velocity[x+3]-np.sign(velocity[x+3])*self.deadbandTorque
        return velocity
    ## Returns a selection vector used for deciding which axises we should move along or rotate around
    # based on if the force/torque in that axis is high enough to overcome the deadband.
    def getDeadbandVector(self, velocity):
        activeDirections = [0,0,0,0,0,0]
        for x in range(3):
            if abs(velocity[x]) > self.deadbandForce:
                activeDirections[x] = 1
        for x in range(3):
            if abs(velocity[x+3]) > self.deadbandTorque:
                activeDirections[x+3] = 1      
        return activeDirections

    '''
    def forceRegulator(self,force):
        Kp=2
        Ki=5
        Kd=1
        T=0.04
        h=1.0/125
        prevForceError=self.forceError
        self.forceError=np.subtract(force, self.forceReference)
        P=Kp*self.forceError
        self.integralPart=self.integralPart+Ki*h*self.forceError
        self.derivativePart=np.multiply(self.derivativePart, T/(T+h))+np.multiply(self.forceError-prevForceError, Kd/(T+h))
        self.forceReference=P+self.integralPart+self.derivativePart

        return self.forceReference
    ''' 

        







        
    

        
        


