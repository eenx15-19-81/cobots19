import numpy as np
import math

from geometry_msgs.msg import TwistStamped, WrenchStamped

class optoForce():
    curForce=[]
    curTorque=[]
    def __init__(self):
        print("Initialized optoForce node")

    def setCurrentForce(self, curForce):
        self.curForce=curForce

    def setCurrentTorque(self,curTorque):
        self.curTorque=curTorque

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
        #TODO rotation_matrix
        return desiredVelocity

    def withinDeadBand(self):

        if(math.sqrt(math.pow(self.curForce[0],2)+math.pow(self.curForce[1],2)+math.pow(self.curForce[2],2))>3):
            return True
        return False
    

        
        


