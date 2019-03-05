import numpy as np
import math
import time


from geometry_msgs.msg import TwistStamped, WrenchStamped

class optoForce():
    curForce=[]
    curTorque=[]
    def __init__(self,tf,rospy):
        self.tf = tf
        self.rospy = rospy
        print("Initialized optoForce class")
        self.listener = tf.TransformListener()

    def setCurrentForce(self, curForce):
        self.curForce=curForce

    def setCurrentTorque(self,curTorque):
        self.curTorque=curTorque

    def transformMatrix(self, frame1, frame2):
        self.listener.waitForTransform( '/'+frame1, '/'+frame2, self.rospy.Time(0),self.rospy.Duration(1))
        (trans,rot) = self.listener.lookupTransform('/'+frame1, '/'+frame2, self.rospy.Time(0))
        return self.listener.fromTranslationRotation(trans, rot)

    def convertFrame(self, velocity):
        rotationMatrix = self.transformMatrix('base','tool0_controller')
        linearVelocity = np.matmul(rotationMatrix[0:3,0:3], velocity[0:3]) 
        velocity = np.concatenate((linearVelocity, np.array([0,0,0]))) 
        return velocity

    def getSpeedl(self, acceleration = 0.3,time=0.5):
        velocity=self.forceControl()
        command = "speedl(" + np.array2string(velocity, precision= 3, separator=',') +","+ \
        str(acceleration) + "," + str(time) + ")" #0.3,0.2
        #rospy.loginfo(velocity)
        return command

    def forceControl(self,kd_inv=0.005):
        if  self.withinDeadBand():
            velocity = np.array(self.curForce)
            #TODO selction_vector
        else:
            velocity = np.array([0,0,0])
        desiredVelocity=np.concatenate((velocity,np.array([0.0,0.0,0.0])))
        desiredVelocity =kd_inv * desiredVelocity
        velocity = self.convertFrame(desiredVelocity)
        return velocity

    def withinDeadBand(self):

        if(math.sqrt(math.pow(self.curForce[0],2)+math.pow(self.curForce[1],2)+math.pow(self.curForce[2],2))>3):
            return True
        return False
    

        
        


