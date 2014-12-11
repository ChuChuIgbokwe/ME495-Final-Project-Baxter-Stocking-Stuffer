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
from std_msgs.msg import (
    String,
    Header,
    Bool,
    Uint8
)

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

tag_id = int()

scanned_ar = None

#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_cd = rospy.Publisher('start/colordetection', Bool) #change to true once you get the stocking position
pub_stocking_pose = rospy.Publisher('stocking_pose',PoseStamped)
pub_move = rospy.Publisher('/baxter_movement/posestamped',PoseStamped)

#initially move to hard coded position
def MovetoFirstStocking():
	'''
	Move to the first stocking. It's position can either be hard coded or input using a private parameter
	'''
    move_to_pose = PoseStamped()
    move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
    move_to_pose.pose.position=Point(
                    x=pointx,# hard code the posiiton and orientation of the first stocking here
                    y=pointy,
                    z=pointz,
                )
    move_to_pose.pose.orientation=Quaternion(
                    x=quatx,# and here
                    y=quaty,
                    z=quatz,
                    w=quatw,
                )
	pub_move.publish(move_to_pose)

def MovetoQRcode():
	'''
	Combine position and orientation in a PoseStamped() message
	'''
    move_to_pose = PoseStamped()
    move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
    move_to_pose.pose.position=Point(
                    x=pose_ee[0,0],
                    y=pose_ee[1,0],
                    z=pose_ee[2,0],	#add the distance between the QR code and the stocking here
                )
    move_to_pose.pose.orientation=Quaternion(
                    x=pose_ee[3,0],
                    y=pose_ee[4,0],
                    z=pose_ee[5,0],
                    w=pose_ee[6,0],
                )
	pub_move.publish(move_to_pose)

	
def MovetoSecondStocking():
	move_to_pose = PoseStamped()
    move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
    move_to_pose.pose.position=Point(
                    x=pose_ee[0,0],
                    y=pose_ee[1,0], # we're moving in the -ve y direction however far the stockings are placed
                    z=pose_ee[2,0],	#whatever we moved up before, we're moving down
                )
    move_to_pose.pose.orientation=Quaternion(
                    x=pose_ee[3,0],
                    y=pose_ee[4,0],
                    z=pose_ee[5,0],
                    w=pose_ee[6,0],
                )
	pub_move.publish(move_to_pose)
		
def ComparePresentIDtoScannedID(currentstocking):
	if scanned_ar == tag_id:
		#Store pose of QR code from camera into local variables
		#PoseStamp messages contains a header and a pose. We care only about the pose so we extract it.
		position_visp = tag_msg.pose.position
		quat_visp = tag_msg.pose.orientation

		#rospy.loginfo("Tag Point Position: [ %f, %f, %f ]"%(position_visp.x, position_visp.y, position_visp.z))
		#rospy.loginfo("Tag Quat Orientation: [ %f, %f, %f, %f]"%(quat_visp.x, quat_visp.y, quat_visp.z, quat_visp.w))

		tag_pos_x = position_visp.x
		tag_pos_y = position_visp.y 
		tag_pos_z = position_visp.z

		tag_quat_x = quat_visp.x
		tag_quat_y = quat_visp.y
		tag_quat_z = quat_visp.z
		tag_quat_w = quat_visp.w


		move_to_pose = PoseStamped()
		move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
		move_to_pose.pose.position=Point(
		                x=tag_pos_x,
		                y=tag_pos_y, #we move to the ar code on the next stocking 
		                z=tag_pos_z,	
		            )
		move_to_pose.pose.orientation=Quaternion(
		                x=tag_quat_x,
		                y=tag_quat_y,
		                z=tag_quat_z,
		                w=tag_quat_w,
		            )
		

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
 
    #print "Position of end-effector=",  pose_ee[0:3,:]

    #Store pose of QR code from camera into local variables
    position_visp = msg.pose.position
    quat_visp = msg.pose.orientation

    #rospy.loginfo("Tag Point Position: [ %f, %f, %f ]"%(position_visp.x, position_visp.y, position_visp.z))
    #rospy.loginfo("Tag Quat Orientation: [ %f, %f, %f, %f]"%(quat_visp.x, quat_visp.y, quat_visp.z, quat_visp.w))


    tag_pos_x = position_visp.x
    tag_pos_y = position_visp.y
    tag_pos_z = position_visp.z

    tag_quat_x = quat_visp.x
    tag_quat_y = quat_visp.y
    tag_quat_z = quat_visp.z
    tag_quat_w = quat_visp.w


    #Determines is there was a change from previous tag position and current
    if math.fabs(tag_pos_x - previous_tag[0,0])>1e-2 and math.fabs(tag_pos_y - previous_tag[0,1])>1e-2 and math.fabs(tag_pos_z - previous_tag[0,2])>1e-1:
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

    #print "Desired position to move to", move_to_pose.pose.position
    #print "Desired orientation to move to", move_to_pose.pose.orientation


    #Update previous_tag with the distance from Baxter's camera to QR code
    global previous_tag
    previous_tag[0,0] = msg.pose.position.x
    previous_tag[0,1] = msg.pose.position.y
    previous_tag[0,2] = msg.pose.position.z


    #Publish PoseStamped() message to move to
    pub.publish(move_to_pose)

    rospy.sleep(2)

    return 

def get_tag_id(msg):
    global get_tag_id
    tag_id = msg.data

def sweep_val(msg):
    global run
    run = msg.data

def read_ar(msg):
	global scanned_ar
	for m in msg.markers:
        scanned_ar = m.id

def main():
    rospy.init_node('create_pose_using_qr_code',anonymous = True)

    #Subscribe to Baxter's left end-effector state and Visp autotracker messages
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getPoseEE)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,getPoseTag)
    rospy.Subscriber("/sweep",Bool,sweep_val)
    rospy.Subscriber("/scanned_stocking_id",Int8,get_tag_id)
	rospy.Subscriber("/ar_pose_marker",AlvarMarkers,read_ar)


    #Wait for left gripper's end effector pose
    while not first_flag:
        pass
	MovetoFirstStocking()

    #Continously running to send PoseStamped() message to Baxter
    while not rospy.is_shutdown():
        NewPoseUsingQRcode(tag_msg)

   
    rospy.spin()



if __name__ == '__main__':
    main()
