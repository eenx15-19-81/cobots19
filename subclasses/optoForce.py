import numpy as np
import math
import time


from geometry_msgs.msg import TwistStamped, WrenchStamped

class optoForce():
    curForce=[]
    curTorque=[]
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

    def getSpeedl(self, acceleration = 0.3,time=0.5):
        velocity=self.forceControl()
        command = "speedl(" + np.array2string(velocity, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")" #0.3,0.2
        #rospy.loginfo(velocity)
        return command

    def forceControl(self,kd_inv=0.005):
        if  self.withinDeadBandForce():
            velocity = np.array(self.curForce)
            
            #TODO selction_vector
        else:
            velocity = np.array([0,0,0])
        if self.withinDeadBandTorque():
            torque= np.array(self.curTorque)
        else:
            torque = np.array([0.0,0.0,0.0])
        velocity =kd_inv * velocity
        desiredVelocity=np.concatenate((velocity,0.2*torque))
        velocity = self.convertFrame(desiredVelocity)
        return velocity

    def withinDeadBandForce(self):

        if(math.sqrt(math.pow(self.curForce[0],2)+math.pow(self.curForce[1],2)+math.pow(self.curForce[2],2))>3):
            return True
        return False

    def withinDeadBandTorque(self):

        if(math.sqrt(math.pow(self.curTorque[0],2)+math.pow(self.curTorque[1],2)+math.pow(self.curTorque[2],2))>3):
            return True
        return False

    def forceRegulator(self,force):#ej fungerande än
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
        

        







        
    

        
        


