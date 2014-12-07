#!/usr/bin/env python

import argparse
import struct
import sys
import numpy as np
import rospy
import baxter_interface
import math

from math import fabs

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String,Header,Bool

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState



#Create publisher to publish center of object detected
pub_color = rospy.Publisher('color_identifier', String, queue_size=10, latch=True)
color = "red"


#Setting flags to False
first_flag = False #position of stocking
second_flag = False #T/F for whether to move back to stocking
third_flag = False #T/F for whether to release present


#Initialization of global variables
StateBacktoStocking = False
StateDropPresent = False

pose_stocking = np.full((7,1), None)



#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('baxter_movement/posestamped', PoseStamped)

pub_state_backtostocking = rospy.Publisher('start/backtostocking', Bool)

pub_state_releasepresent = rospy.Publisher('start/releasepresent', Bool)

pub_state_sweepstocking = rospy.Publisher('sweep', Bool)



#Obtain T/F state of whether to move back to stocking
def getStateBackToStocking(msg):

    global StateBacktoStocking

    StateBacktoStocking = msg.data

    second_flag = True



#Obtain T/F state of whether to release present
def getStateReleasePresent(msg):

    global StateRealeasePresent

    StateRealeasePresent = msg.data

    third_flag = True



#Obtains the pose of the stocking from a prior node
def getPoseStocking(msg):

    pose = msg.pose

    position_new = pose.position
    orientation_new = pose.orientation

    global pose_stocking, first_flag

    pose_stocking[0,0] = position_new.x
    pose_stocking[1,0] = position_new.y
    pose_stocking[2,0] = position_new.z

    pose_stocking[3,0] = orientation_new.x
    pose_stocking[4,0] = orientation_new.y
    pose_stocking[5,0] = orientation_new.z
    pose_stocking[6,0] = orientation_new.w

    first_flag = True

    return




#Creates PoseStamped() message to move back to stocking
def NewPoseBacktoStocking(msg):

    #Wait for T/F state of whether to move back to stocking
    while not second_flag:
        pass
 
    #Only obtain a new pose if asked to
    if StateBacktoStocking == "True":

        #Stores pose of the stocking in a local variable
        pose_stocking = msg


        #Combine position and orientation in a PoseStamped() message
        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
        move_to_pose.pose.position=Point(
                        x=pose_stocking[0,0],
                        y=pose_stocking[1,0],
                        z=pose_stocking[2,0],
                    )
        move_to_pose.pose.orientation=Quaternion(
                        x=pose_stocking[3,0],
                        y=pose_stocking[4,0],
                        z=pose_stocking[5,0],
                        w=pose_stocking[6,0],
                    )
        
        #Send PoseStamped() message to Baxter's movement function
        pub_baxtermovement.publish(move_to_pose)

        #Turn state of back to stocking to False and state of release present to True
        pub_state_backtostocking.publish("False")

        pub_state_releasepresent.publish("True")


        rospy.sleep(2)


    return 



#Release present by opening gripper
def ReleasePresent():

    #Wait for T/F state of whether to open gripper
    while not third_flag:
        pass
 
    #Only release present if asked to
    if StateDropPresent == "True":

            #Open Baxter's left gripper
            baxterleft = baxter_interface.Gripper('left')
            baxterleft.open()

            #Wait for gripper to open
            rospy.sleep(1)

            #Once released, turn state to F, and change T/F state of sweep to T
            pub_state_releasepresent.publish("False")

            pub_state_sweepstocking.publish("True")


    return






#Main section of loop
def main():

    rospy.init_node('create_pose_back_to_stocking',anonymous = True)


    #Subscribe to pose of stocking and T/F topics of whether to move back to stocking and release present
    rospy.Subscriber("/pose/stocking",PoseStamped,getPoseStocking)
    rospy.Subscriber("/start/backtostocking",Bool,getStateBackToStocking)
    rospy.Subscriber("/start/releasepresent",Bool,getStateReleasePresent)


    #Wait for pose of stocking
    while not first_flag:
        pass


    #Send PoseStamped() message to Baxter and open gripper
    while not rospy.is_shutdown():
        NewPoseBacktoStocking(pose_stocking)
        ReleasePresent()

   
    rospy.spin()



if __name__ == '__main__':
    main()