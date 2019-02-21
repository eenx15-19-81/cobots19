#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eros
    # ROS Intro Course Exercise
    # V.1.0.0.

    # Every line of code that is commented away right now is code from move_robot.py.
    # This code only adds objects.
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import sys
import time
import numpy
import tf
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import PlanningSceneInterface as psi

class add_objects():

    def __init__(self):

        # ROS node initializer:
        rospy.init_node('move_robot', anonymous=False)

        # Move Group specifier:
        #self.robot = mgc("manipulator")
        self.scene = psi()

        # Message type initializers:
        self.pose_stamped = PoseStamped()
       
        # Some time to assure initialization:
        rospy.sleep(2)

        # Add some objects in the scene
        #self.add_object("TOOL", [0.5, -0.25, 1.5, 0, 0, 0], (0.1, 0.1, 0.1))
        self.add_object("TABLE", [0, 0, -0.5, 0, 0, 0], (1, 1, 1))
        #self.add_object("ITEM", [-0.7, -0.2, 1.1, 0, 0, 0], (0.2, 0.2, 0.2))
        self.add_object("HUMAN", [1, 0.6, 1, 3, 1, 1], (0.6, 0.3, 2))

        rospy.sleep(1)

        self.added_objects = self.scene.get_known_object_names()

        self.main()


    def main(self):

        rospy.spin()


    def switcher(self, what, case_list):

        for i in range(0, len(case_list), 1):
            if what == case_list[i]:
                return i
                break
            else:
                pass


    def rpy_to_quat(self, x, y, z, roll, pitch, yaw):
     
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        qx = quaternion[0]
        qy = quaternion[1]
        qz = quaternion[2]
        qw = quaternion[3]

        return [x, y, z, qx, qy, qz, qw]
    

    def add_object(self, name, rpy_pose, size):

        quat = self.rpy_to_quat(rpy_pose[0], rpy_pose[1], rpy_pose[2], rpy_pose[3], rpy_pose[4], rpy_pose[5])

        self.pose_stamped.header.frame_id = "world"
        self.pose_stamped.pose.position.x = quat[0]
        self.pose_stamped.pose.position.y = quat[1]
        self.pose_stamped.pose.position.z = quat[2]
        self.pose_stamped.pose.orientation.x = quat[3]
        self.pose_stamped.pose.orientation.y = quat[4]
        self.pose_stamped.pose.orientation.z = quat[5]
        self.pose_stamped.pose.orientation.w = quat[6]

        self.scene.add_box(name, self.pose_stamped, size)


if __name__ == '__main__':
    try:
        add_objects()
    except rospy.ROSInterruptException:
        pass
