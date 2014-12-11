#!/usr/bin/env python

import argparse
import struct
import sys
import numpy as np
import rospy
import baxter_interface
import math

from math import fabs,sqrt

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String,Header,Int8,Bool

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker




#Setting flags to False
first_flag = False #position of end-effector
second_flag = False #QR pose
third_flag = False #goal ID to be looking for
fourth_flag = False #current ID found



#Initialization of global variables
tag_msg = PoseStamped()

pose_ee = np.full((7,1), None)

pose_present = np.full((7,1), None)

previous_tag = np.zeros((1,3))

movement = 0 #QR code movement (0,1,2)

goal_ID = None

previous_ID = None

StateStockingPose = None

iteration = 0



#Setting start pose to look for goal ID
pose_start = np.full((7,1), None)
pose_start[0,0] = -.5
pose_start[1,0] = 0.9
pose_start[2,0] = 0.2
pose_start[3,0] = -0.5
pose_start[4,0] = 0.5
pose_start[5,0] = 0.5
pose_start[6,0] = 0.5



#Once found pose of stocking, initial pose above table to start looking for colored object
pose_above_table = np.full((7,1), None)
pose_above_table[0,0] = 0.8
pose_above_table[1,0] = 0.15
pose_above_table[2,0] = 0.35
pose_above_table[3,0] = 0
pose_above_table[4,0] = 1
pose_above_table[5,0] = 0
pose_above_table[6,0] = 0


#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('baxter_movement/posestamped', PoseStamped)

#Create publisher to store PoseStamped() message of stocking for when returning to drop present
pub_statestocking = rospy.Publisher('start/stockingpose', Bool)
pub_posestocking = rospy.Publisher('pose/stocking', PoseStamped)

pub_statecolordetection = rospy.Publisher('start/colordetection', Bool)



def getStateStockingPose(msg):

    global StateStockingPose

    StateStockingPose = msg.data

    return



#Gets pose of end-effector from Baxter
def getPoseEE(msg):

    pose = msg.pose

    position_new = pose.position
    orientation_new = pose.orientation

    global pose_ee, first_flag

    pose_ee[0,0] = position_new.x
    pose_ee[1,0] = position_new.y
    pose_ee[2,0] = position_new.z

    pose_ee[3,0] = orientation_new.x
    pose_ee[4,0] = orientation_new.y
    pose_ee[5,0] = orientation_new.z
    pose_ee[6,0] = orientation_new.w

    first_flag = True

    return



#Gets the pose of tag from QR code
def getPoseTag(msg):

    global tag_msg, second_flag

    tag_msg = msg

    second_flag = True

    return


#Gets the goal ID to be looking for
def getStockingID(msg):

    global goal_ID, third_flag

    goal_ID = msg.data

    third_flag = True

    return



#Get the current ID found
def getTagID(msg):

    global current_ID, fourth_flag

    scanned_identity = []

    #Determine current ID
    for m in msg.markers:
        identity = m.id

        scanned_identity.append(identity)

    #If found more than one identity, picks the first one (usually left-most)
    if len(scanned_identity) != 0:

        if len(scanned_identity)==1 or len(scanned_identity)==2:
            current_ID = scanned_identity[0]

        elif len(scanned_identity) == 3:
            current_ID = scanned_identity[1]

        elif len(scanned_identity) == 4:
            current_ID = scanned_identity[2]

        scanned_identity = np.sort(scanned_identity)

        #Set current ID to a global variable, and turn state of fourth_flag to True


        fourth_flag = True


    return




#Create PoseStamped() message to move Baxter towards QR code
def FindCorrectID():

    #Wait for pose of QR code, and ID numbers to be looking for / currently found
    while not second_flag:
        while not third_flag:
            while not fourth_flag:
                pass
 
    rospy.sleep(1)

    #Global variables being used within this section
    global pose_present, previous_ID, iteration

    #Local variables for determining movement between stockings
    stocking_distance_apart = 0.23 #m
    dist_qr_above_stocking = 0.2 #m
    wait = 0

    
    #Only run when asked to
    if StateStockingPose == True:

        print "State of Stocking Pose:",StateStockingPose,iteration

        #Move to start pose
        if iteration == 0:
            print "Moving to start pose"
            pointx = pose_start[0,0]
            pointy = pose_start[1,0]
            pointz = pose_start[2,0]
          
            quatx = pose_start[3,0]
            quaty = pose_start[4,0]
            quatz = pose_start[5,0]
            quatw = pose_start[6,0]

        #At first stocking, read ID, move above stocking to scan QR code
        elif iteration == 1:

            print "At first stocking, reading ID, move above stocking to scan QR code"

            #Scanned ID is the goal ID
            if current_ID == goal_ID:
               previous_ID = current_ID

            print "Current_ID:",current_ID
            #Move to above stocking to read QR code
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0] + 0.15
            pointz = pose_ee[2,0] + dist_qr_above_stocking
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

        #Above first stocking, read QR code, store it for later, and move to second stocking
        elif iteration == 2:

            print "Above first stocking, read QR code, store it for later, and move to second stocking"

            #Obtain pose of QR code
            position_visp = tag_msg.pose.position
            quat_visp = tag_msg.pose.orientation

            tag_pos_x = position_visp.x
            tag_pos_y = position_visp.y
            tag_pos_z = position_visp.z

            tag_quat_x = quat_visp.x
            tag_quat_y = quat_visp.y
            tag_quat_z = quat_visp.z
            tag_quat_w = quat_visp.w

            print "QR Code Position from EE Pose:",position_visp
            print "Current E-E Pose:",pose_ee

            #Store position of QR code as pose of present
            pose_present[0,0] = pose_ee[0,0] + tag_pos_x
            pose_present[1,0] = pose_ee[1,0] + tag_pos_z
            pose_present[2,0] = pose_ee[2,0] + tag_pos_y

            pose_present[3,0] = -0.5
            pose_present[4,0] = 0.5
            pose_present[5,0] = 0.5
            pose_present[6,0] = 0.5

            print "Pose of the present:",pose_present

            #Move to second stocking
            pointx = pose_ee[0,0] + stocking_distance_apart
            pointy = pose_ee[1,0] - 0.15
            pointz = pose_ee[2,0] - dist_qr_above_stocking
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

        #At second stocking, read ID, move to third stocking
        elif iteration == 3:

            print "At second stocking, read ID, move to third stocking"

            #Scanned ID is the goal ID
            if current_ID == goal_ID:
                iteration = 1000 #exit loop

            print "Current_ID:",current_ID

            #Store position of present to above second stocking
            pose_present[0,0] = pose_present[0,0] + stocking_distance_apart
            pose_present[1,0] = pose_present[1,0]
            pose_present[2,0] = pose_present[2,0]

            pose_present[3,0] = pose_present[3,0]
            pose_present[4,0] = pose_present[4,0]
            pose_present[5,0] = pose_present[5,0]
            pose_present[6,0] = pose_present[6,0]

            #Move to third stocking
            pointx = pose_ee[0,0] + stocking_distance_apart
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0]
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

            #If 1st ID was goal ID (did not have pose of QR code at that time)
            if previous_ID == goal_ID:

                #Update pose of present
                pose_present[0,0] = pose_present[0,0] - stocking_distance_apart
                pose_present[1,0] = pose_present[1,0]
                pose_present[2,0] = pose_present[2,0]

                pose_present[3,0] = pose_present[3,0]
                pose_present[4,0] = pose_present[4,0]
                pose_present[5,0] = pose_present[5,0]
                pose_present[6,0] = pose_present[6,0]

                #Update pose of end-effector such that it does not move
                pointx = pose_ee[0,0]
                pointy = pose_ee[1,0]
                pointz = pose_ee[2,0]
              
                quatx = pose_ee[3,0]
                quaty = pose_ee[4,0]
                quatz = pose_ee[5,0]
                quatw = pose_ee[6,0]

                iteration = 1000 #exit loop


        #At third stocking, read ID, move to fourth stocking
        elif iteration == 4:

            print "At third stocking, read ID, move to fourth stocking"

            #Scanned ID is goal ID
            if current_ID == goal_ID:
                iteration = 1000 #exit loop

            print "Current_ID:",current_ID

            #Store position of present to above third stocking
            pose_present[0,0] = pose_present[0,0] + stocking_distance_apart
            pose_present[1,0] = pose_present[1,0]
            pose_present[2,0] = pose_present[2,0]

            pose_present[3,0] = pose_present[3,0]
            pose_present[4,0] = pose_present[4,0]
            pose_present[5,0] = pose_present[5,0]
            pose_present[6,0] = pose_present[6,0]

            #Move to fourth stocking
            pointx = pose_ee[0,0] + stocking_distance_apart
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0]
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

        #At fourth stocking, read ID
        elif iteration == 5:

            print "At fourth stocking, read ID"

            #Scanned ID is goal ID
            if current_ID == goal_ID:
                iteration = 1000 #exit loop

            print "Current_ID:",current_ID

            #Store position of present to above fourth stocking
            pose_present[0,0] = pose_present[0,0] + stocking_distance_apart
            pose_present[1,0] = pose_present[1,0]
            pose_present[2,0] = pose_present[2,0]

            pose_present[3,0] = pose_present[3,0]
            pose_present[4,0] = pose_present[4,0]
            pose_present[5,0] = pose_present[5,0]
            pose_present[6,0] = pose_present[6,0]

            #Remain stationary
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0]
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

        #Goal ID was not found, try again by starting at home position
        elif iteration == 6:
            iteration = -1
            print "Goal ID was not found, try again by starting at home position"

            #Remain stationary
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0]
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

        #Goal ID was correctly found, publish pose of stocking for later use after finding correct present
        elif iteration > 1000:

            print "Goal ID was correctly found, publish pose of stocking for later use after finding correct present"

            #Move Baxter to a position above the table where he can see all potential presents
            pointx = pose_above_table[0,0]
            pointy = pose_above_table[1,0]
            pointz = pose_above_table[2,0]
          
            quatx = pose_above_table[3,0]
            quaty = pose_above_table[4,0]
            quatz = pose_above_table[5,0]
            quatw = pose_above_table[6,0]

            #Publish pose of present
            move_to_present = PoseStamped()
            move_to_present.header=Header(stamp=rospy.Time.now(), frame_id='base')
            move_to_present.pose.position=Point(
                            x=pose_present[0,0],
                            y=pose_present[1,0],
                            z=pose_present[2,0],
                        )
            move_to_present.pose.orientation=Quaternion(
                            x=pose_present[3,0],
                            y=pose_present[4,0],
                            z=pose_present[5,0],
                            w=pose_present[6,0],
                        )

            print "Pose of present:",move_to_present.pose
            print "Moving to pick up present."

            pub_posestocking.publish(move_to_present)

            #Once found stocking, turn status to F and start up next node to move towards object
            pub_statestocking.publish(False)

            wait = 1000

            iteration = -1


        #Increment iteration
        iteration = iteration + 1

        #Combine position and orientation in a PoseStamped() message
        move_to_pose = PoseStamped()
        move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
        move_to_pose.pose.position=Point(
                        x=pointx,
                        y=pointy,
                        z=pointz,
                    )
        move_to_pose.pose.orientation=Quaternion(
                        x=quatx,
                        y=quaty,
                        z=quatz,
                        w=quatw,
                    )

        #Publish PoseStamped() message to move to
        pub_baxtermovement.publish(move_to_pose)


    #Wait for pose_ee to get updated correctly
    global first_flag
    first_flag = False

    rospy.sleep(6)

    if wait == 1000:
        print "About to sleep for 8 seconds"
        rospy.sleep(4)
        pub_statecolordetection.publish(True)
        print "Slept for 8 seconds"

    return 




#Main section of code to run
def main():

    rospy.init_node('create_pose_using_qr_code',anonymous = True)


    #Subscribe to Baxter's left end-effector state, Visp/AR autotracker messages, and stocking ID to be looking for
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getPoseEE)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,getPoseTag)
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers,getTagID)
    rospy.Subscriber("/scanned_stocking_id",Int8,getStockingID)
    rospy.Subscriber("/start/stockingpose",Bool,getStateStockingPose)


    #Wait for left gripper's end effector pose
    while not first_flag:
        pass


    #Continously running to send PoseStamped() message to Baxter
    while not rospy.is_shutdown():
        FindCorrectID()

   
    rospy.spin()



if __name__ == '__main__':
    main()