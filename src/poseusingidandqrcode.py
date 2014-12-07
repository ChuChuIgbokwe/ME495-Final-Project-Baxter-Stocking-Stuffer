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
from std_msgs.msg import String,Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState




#Setting flags to False
first_flag = False #position of end-effector
second_flag = False #QR pose
third_flag = True #goal ID to be looking for
fourth_flag = True #current ID found



#Initialization of global variables
tag_msg = PoseStamped()

pose_ee = np.full((7,1), None)

pose_present = np.full((7,1), None)

previous_tag = np.zeros((1,3))

movement = 0 #QR code movement (0,1,2)


#Setting start pose to look for goal ID
pose_start = np.full((7,1), None)
pose_start[0,0] = 0
pose_start[1,0] = 0
pose_start[2,0] = 0
pose_start[3,0] = 0
pose_start[4,0] = 0
pose_start[5,0] = 0
pose_start[6,0] = 0



#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('baxter_movement/posestamped', PoseStamped)




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

    goal_ID = msg

    third_flag = True

    return



#Get the current ID found
def getTagID(msg):

    global current_ID, fourth_flag

    #Determine current ID
    for m in msg.markers:
        identity = m.id

        print "Identity:",identity
        print "Pose:","\n",m.pose.pose.position,"\n"


    #Set current ID to a global variable, and turn state of fourth_flag to True
    current_ID = identity

    fourth_flag = True


    return




#Create PoseStamped() message to move Baxter towards QR code
def FindCorrectID():

    #Wait for pose of QR code, and ID numbers to be looking for / currently found
    while not second_flag:
        pass
    while not third_flag:
        pass
    while not fourth_flag:
        pass
 
    global pose_present, previous_ID

    #Move to start pose
    if iteration == 0:
        pointx = pose_start[0,0]
        pointy = pose_start[1,0]
        pointz = pose_start[2,0]
      
        quatx = pose_start[3,0]
        quaty = pose_start[4,0]
        quatz = pose_start[5,0]
        quatw = pose_start[6,0]

    #At first stocking, read ID, move above stocking to scan QR code
    elif iteration == 1:

        if current_ID == goal_ID:
           correct id found
           previous_id = current_ID
        else:
            id not found

        #Move to above stocking to read QR code
        pointx = pose_ee[0,0]
        pointy = pose_ee[1,0]
        pointz = pose_ee[2,0] + 0.05
      
        quatx = pose_ee[3,0]
        quaty = pose_ee[4,0]
        quatz = pose_ee[5,0]
        quatw = pose_ee[6,0]

    #Above first stocking, read QR code, store it for later, and move to second stocking
    elif iteration == 2:

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

        #Store position of QR code as pose of present
        pose_present[0,0] = pose_ee[0,0] - tag_pos_x
        pose_present[1,0] = pose_ee[1,0] - tag_pos_y
        pose_present[2,0] = pose_ee[2,0] - tag_pos_z

        pose_present[3,0] = 
        pose_present[4,0] = 
        pose_present[5,0] = 
        pose_present[6,0] = 

        #Move to second stocking
        pointx = pose_ee[0,0]
        pointy = pose_ee[1,0] - 0.1
        pointz = pose_ee[2,0] - 0.05
      
        quatx = pose_ee[3,0]
        quaty = pose_ee[4,0]
        quatz = pose_ee[5,0]
        quatw = pose_ee[6,0]

    #At second stocking, read ID, move to third stocking
    elif iteration == 3:

        if current_ID == goal_ID:
            correct id found
            iteration = 1000 #exit loop

        else:
            id not found

        #Store position of present to above second stocking
        pose_present[0,0] = pose_present[0,0]
        pose_present[1,0] = pose_present[1,0]
        pose_present[2,0] = pose_present[2,0] + 0.05

        pose_present[3,0] = pose_present[3,0]
        pose_present[4,0] = pose_present[4,0]
        pose_present[5,0] = pose_present[5,0]
        pose_present[6,0] = pose_present[6,0]

        #Move to third stocking
        pointx = pose_ee[0,0]
        pointy = pose_ee[1,0] - 0.1
        pointz = pose_ee[2,0]
      
        quatx = pose_ee[3,0]
        quaty = pose_ee[4,0]
        quatz = pose_ee[5,0]
        quatw = pose_ee[6,0]

        #If 1st ID was goal ID (did not have pose of QR code at that time)
        if previous_ID == goal_ID:
            correct id was at first stocking

            #Update pose of present
            pose_present[0,0] = pose_present[0,0]
            pose_present[1,0] = pose_present[1,0]
            pose_present[2,0] = pose_present[2,0] - 0.05

            pose_present[3,0] = pose_present[3,0]
            pose_present[4,0] = pose_present[4,0]
            pose_present[5,0] = pose_present[5,0]
            pose_present[6,0] = pose_present[6,0]

            #Update pose of end-effector such that it does not move
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0] - 0.1
            pointz = pose_ee[2,0]
          
            quatx = pose_ee[3,0]
            quaty = pose_ee[4,0]
            quatz = pose_ee[5,0]
            quatw = pose_ee[6,0]

            iteration = 1000 #exit loop


    #At third stocking, read ID, move to fourth stocking
    elif iteration == 4:

        if current_ID == goal_ID:
            correct id found
            iteration = 1000 #exit loop
        else:
            id not found

        #Store position of present to above third stocking
        pose_present[0,0] = pose_present[0,0]
        pose_present[1,0] = pose_present[1,0]
        pose_present[2,0] = pose_present[2,0] + 0.05

        pose_present[3,0] = pose_present[3,0]
        pose_present[4,0] = pose_present[4,0]
        pose_present[5,0] = pose_present[5,0]
        pose_present[6,0] = pose_present[6,0]

        #Move to fourth stocking
        pointx = pose_ee[0,0]
        pointy = pose_ee[1,0] - 0.1
        pointz = pose_ee[2,0]
      
        quatx = pose_ee[3,0]
        quaty = pose_ee[4,0]
        quatz = pose_ee[5,0]
        quatw = pose_ee[6,0]

    #At fourth stocking, read ID
    elif iteration == 5:

        if current_ID == goal_ID:
            correct id found
            iteration = 1000 #exit loop
        else:
            id not found

        #Store position of present to above fourth stocking
        pose_present[0,0] = pose_present[0,0]
        pose_present[1,0] = pose_present[1,0]
        pose_present[2,0] = pose_present[2,0] + 0.05

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
        ID not found
        iteration == -1:

    #Goal ID was correctly found, publish pose of stocking for later use after finding correct present
    elif iteration > 1000:

        #Remain stationary, account for movement to next stocking,
        pointy = pointy - 0.05

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

        pub_posestocking.publish(move_to_present)


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
    rospy.sleep(2)

    return 





def main():
    rospy.init_node('create_pose_using_qr_code',anonymous = True)


    #Subscribe to Baxter's left end-effector state, Visp/AR autotracker messages, and stocking ID to be looking for
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getPoseEE)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,getPoseTag)
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers,getTagID)
    rospy.Subscriber("/stocking_id",PoseStamped,getStockingID)


    #Wait for left gripper's end effector pose
    while not first_flag:
        pass


    #Continously running to send PoseStamped() message to Baxter
    while not rospy.is_shutdown():
        FindCorrectID()

   
    rospy.spin()



if __name__ == '__main__':
    main()