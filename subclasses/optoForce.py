#! /usr/bin/python
import numpy as np
import math
import time


from geometry_msgs.msg import TwistStamped, WrenchStamped

class optoForce():
    curForce=[]
    curTorque=[]

    # Increase if the robot "wanders" when in forcecontrol
    deadbandForce=3
    deadbandTorque=0.5

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

    def transformMatrix(self, frame1, frame2):
        self.listener.waitForTransform( '/'+frame1, '/'+frame2, self.rospy.Time(),self.rospy.Duration(.1))
        (trans,rot) = self.listener.lookupTransform('/'+frame1, '/'+frame2, self.rospy.Time(0))
        return self.listener.fromTranslationRotation(trans, rot)

    def convertFrame(self, velocity):
        rotationMatrix = self.transformMatrix('base','tool0_controller')
        linearVelocity = np.matmul(rotationMatrix[0:3,0:3], velocity[0:3]) 
        angularVelocity = np.matmul(rotationMatrix[0:3,0:3], velocity[3:6])
        velocity = np.concatenate((linearVelocity, angularVelocity)) 
        return velocity

    def getSpeedl(self, acceleration = 0.6, rotAcceleration=1.2, time=0.2):
        velocity=self.forceControl()
        command = "speedl(" + np.array2string(velocity, precision= 3, separator=',') +", "+ \
        str(acceleration) + ", " + str(time) + ", " + str(rotAcceleration) +")" 
        self.rospy.loginfo(command)
        return command

    def forceControl(self, kp_force=0.01, kp_torque=0.2):
        force = np.array(self.curForce)
        torque = np.array(self.curTorque)
        #TODO selction_vector
        force = kp_force * force
        torque = kp_torque * torque
        velocity = np.concatenate([force, torque])
        velocity = np.multiply(velocity, np.array(self.getDeadbandVector()))
        velocity = self.convertFrame(velocity)
        return velocity

    def getDeadbandVector(self):
        activeDirections = [0,0,0,0,0,0]
        for x in range(3):
            if abs(self.curForce[x]) > self.deadbandForce:
                activeDirections[x] = 1
        for x in range(3):
            if abs(self.curTorque[x]) > self.deadbandTorque:
                activeDirections[x+3] = 1      
        return activeDirections

    def withinDeadBandTorque(self):
        if(math.sqrt(math.pow(self.curTorque[0],2)+math.pow(self.curTorque[1],2)+math.pow(self.curTorque[2],2))<self.deadbandTorque):
            return True
        return False

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
        

        







        
    

        
        


