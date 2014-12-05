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



#Initialization of global variables
tag_msg = PoseStamped()

pose_ee = np.full((7,1), None)

previous_tag = np.zeros((1,3))

movement = 0 #QR code movement (0,1,2)



#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub = rospy.Publisher('baxter_movement/posestamped', PoseStamped)




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




#Create PoseStamped() message to move Baxter towards QR code
def NewPoseUsingQRcode(msg):

    while not second_flag:
        pass
 
    
    print "Position of end-effector=",  pose_ee[0:3,:]

    #Store pose of QR code from camera into local variables
    position_visp = msg.pose.position
    quat_visp = msg.pose.orientation

    rospy.loginfo("Tag Point Position: [ %f, %f, %f ]"%(position_visp.x, position_visp.y, position_visp.z))
    rospy.loginfo("Tag Quat Orientation: [ %f, %f, %f, %f]"%(quat_visp.x, quat_visp.y, quat_visp.z, quat_visp.w))


    tag_pos_x = position_visp.x
    tag_pos_y = position_visp.y
    tag_pos_z = position_visp.z

    tag_quat_x = quat_visp.x
    tag_quat_y = quat_visp.y
    tag_quat_z = quat_visp.z
    tag_quat_w = quat_visp.w


    #Determines is there was a change from previous tag position and current
    if math.fabs(tag_pos_x - previous_tag[0,0])<1e-2 and math.fabs(tag_pos_y - previous_tag[0,1])<1e-2 and math.fabs(tag_pos_z - previous_tag[0,2])<1e-1:
        print "NO movement"
    else:
        global movement
        movement = movement+1

   
    #Account for changes in tag position and only move towards the tag once
    if movement == 0:
        tag_pos_x = 0
        tag_pos_y = 0
        tag_pos_z = 0
        tag_quat_x = 0
        tag_quat_y = 0
        tag_quat_z = 0
        tag_quat_w = 1

    elif movement == 2:
        movement = 0

        tag_pos_x = 0
        tag_pos_y = 0
        tag_pos_z = 0
        tag_quat_x = 0
        tag_quat_y = 0
        tag_quat_z = 0
        tag_quat_w = 1


    #New pose to move towards
    pointx = pose_ee[0,0] - tag_pos_x
    pointy = pose_ee[1,0] - tag_pos_y
    pointz = pose_ee[2,0] - tag_pos_z
  
    quatx = pose_ee[3,0]# - tag_quat_x
    quaty = pose_ee[4,0]# - tag_quat_y
    quatz = pose_ee[5,0]# - tag_quat_z
    quatw = pose_ee[6,0]# - tag_quat_w


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

    print "Desired position to move to", move_to_pose.pose.position
    print "Desired orientation to move to", move_to_pose.pose.orientation


    #Update previous_tag with the distance from Baxter's camera to QR code
    global previous_tag
    previous_tag[0,0] = msg.pose.position.x
    previous_tag[0,1] = msg.pose.position.y
    previous_tag[0,2] = msg.pose.position.z


    #Publish PoseStamped() message to move to
    pub.publish(move_to_pose)

    return 





def main():
    rospy.init_node('create_pose_using_qr_code',anonymous = True)


    #Subscribe to Baxter's left end-effector state and Visp autotracker messages
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getPoseEE)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,getPoseTag)


    #Wait for left gripper's end effector pose
    while not first_flag:
        pass


    #Continously running to send PoseStamped() message to Baxter
    while not rospy.is_shutdown():
        NewPoseUsingOpenCV(tag_msg)

   
    rospy.spin()



if __name__ == '__main__':
    main()