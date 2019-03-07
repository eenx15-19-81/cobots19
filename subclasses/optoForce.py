#! /usr/bin/python
import numpy as np
import math
import time


from geometry_msgs.msg import TwistStamped, WrenchStamped

class optoForce():
    curForce=[]
    curTorque=[]

    # Increase if the robot "wanders" when in forcecontrol
    deadbandForce=10
    deadbandTorque=2.0

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
        str(acceleration) + ", " + str(time) + ", " + rotAcceleration +")" 
        self.rospy.loginfo(command)
        return command

    def forceControl(self, kp_force=0.005, kp_torque=0.2):
        if not self.withinDeadBandForce():
            velocity = np.array(self.curForce)
            velocity = velocity - self.deadbandForce    # Start from zero
        else:
            velocity = np.array([0.0,0.0,0.0])
        if not self.withinDeadBandTorque():
            torque = np.array(self.curTorque)
            torque = torque - self.deadbandTorque   # Start from zero
        else:
            torque = np.array([0.0,0.0,0.0])
        #TODO selction_vector
        velocity = kp_force * velocity
        torque = kp_torque * torque
        desiredVelocity=np.concatenate((velocity,torque))
        velocity = self.convertFrame(desiredVelocity)
        return velocity

    def withinDeadBandForce(self):
        if(math.sqrt(math.pow(self.curForce[0],2)+math.pow(self.curForce[1],2)+math.pow(self.curForce[2],2))<self.deadbandForce):
            return True
        return False

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
        

        







        
    

        
        


