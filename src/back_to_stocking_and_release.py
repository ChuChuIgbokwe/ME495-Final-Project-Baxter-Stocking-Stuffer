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



#Setting flags to False
first_flag = False #position of stocking
second_flag = False #T/F for whether to move back to stocking
third_flag = False #T/F for whether to release present


#Initialization of global variables
StateBacktoStocking = False
StateReleasePresent = False

pose_stocking = np.full((7,1), None)



#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('baxter_movement/posestamped', PoseStamped)

pub_state_backtostocking = rospy.Publisher('start/backtostocking', Bool)

pub_state_releasepresent = rospy.Publisher('start/releasepresent', Bool)

pub_state_sweepstocking = rospy.Publisher('/start/sweep', Bool)



pub_stockingpose = rospy.Publisher('pose/stocking', PoseStamped)




pointx = None
pointy = None
pointz = None




#Obtain T/F state of whether to move back to stocking
def getStateBackToStocking(msg):

    global StateBacktoStocking, second_flag

    StateBacktoStocking = msg.data

    second_flag = True

    return



#Obtain T/F state of whether to release present
def getStateReleasePresent(msg):

    global StateReleasePresent, third_flag

    StateReleasePresent = msg.data

    third_flag = True

    print "StateReleasePresent:",third_flag

    return



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
    if StateBacktoStocking == True:

        #Stores pose of the stocking in a local variable
        pose_stocking = msg



        print "Pose of Stocking:",pose_stocking

        pointx = pose_stocking[0,0]
        pointy = pose_stocking[1,0]
        pointz = pose_stocking[2,0]


        if fabs(pose_stocking[0,0]-(-0.55))<0.1:
            pointx = -0.55
            pointy = 1.06
            pointz = 0.35
        elif fabs(pose_stocking[0,0]-(-0.55+1*0.23))<0.1:
            pointx = -0.55+1*0.23
            pointy = 1.06
            pointz = 0.35
        elif fabs(pose_stocking[0,0]-(-0.55+2*0.23))<0.1:
            pointx = -0.55+2*0.23
            pointy = 1.06
            pointz = 0.35
        elif fabs(pose_stocking[0,0]-(-0.55+3*0.23))<0.1:
            pointx = -0.55+3*0.23
            pointy = 1.06
            pointz = 0.35


        #Combine position and orientation in a PoseStamped() message
        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
        move_to_pose.pose.position=Point(
                        x=pointx,
                        y=pointy,
                        z=pointz,
                    )
        move_to_pose.pose.orientation=Quaternion(
                        x=pose_stocking[3,0],
                        y=pose_stocking[4,0],
                        z=pose_stocking[5,0],
                        w=pose_stocking[6,0],
                    )
        

        print "Moving back to stocking to release present, with pose:"
        print move_to_pose

        #Send PoseStamped() message to Baxter's movement function
        pub_baxtermovement.publish(move_to_pose)


        #Turn state of back to stocking to False and state of release present to True
        pub_state_backtostocking.publish(False)

        rospy.sleep(15)

        print "Should be at location to drop present."

        pub_state_releasepresent.publish(True)



    return 



#Release present by opening gripper
def ReleasePresent():

    #Wait for T/F state of whether to open gripper
    while not third_flag:
        pass
 
    #Only release present if asked to
    if StateReleasePresent == True:

            print "Releasing present in stocking."

            #Open Baxter's left gripper
            baxterleft = baxter_interface.Gripper('left')
            rospy.sleep(1)
            baxterleft.open()

            #Wait for gripper to open
            # rospy.sleep(2)

            print "Present dropped in stocking."

            #Once released, turn state to F, and change T/F state of sweep to T
            pub_state_releasepresent.publish(False)

            rospy.sleep(4)

            #Move to a position where can see all stockings
            #MovetoScanStockingRange()

            pub_state_sweepstocking.publish(True)


    return


#Move to a position where can see all stockings
def MovetoScanStockingRange():

    #Send Baxter back to starting point to find next stocking
    move_to_pose = PoseStamped()
    move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
    move_to_pose.pose.position=Point(
                    x=-0.25,
                    y=0.7,
                    z=0.3,
                )
    move_to_pose.pose.orientation=Quaternion(
                    x=-0.5,
                    y=0.5,
                    z=0.5,
                    w=0.5,
                )
    
    #Send PoseStamped() message to Baxter's movement function
    pub_baxtermovement.publish(move_to_pose)

    rospy.sleep(10)

    global first_flag
    first_flag = True

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