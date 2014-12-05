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


#Create publisher to publish center of object detected
pub_color = rospy.Publisher('color_identifier', String)


#Setting flags to False
first_flag = False #position end-effector
second_flag = False #opencv position


#Initialization of global variables
pose_ee = np.full((7,1), None)

position_object_opencv = Point()


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




#Gets position of object from OpenCV node
def getPositionObjectfromOpenCV(msg):

    global position_object_opencv, second_flag

    position_object_opencv = msg

    second_flag = True

    return




#Creates PoseStamped() message to move appropriately towards object, based on distance away
def NewPoseUsingOpenCV(msg):
 
    
    #Wait for position of object from OpenCV node
    while not second_flag:
        pass
    


    #Store position of object from OpenCV into local variables
    position_OpenCV = msg

    color_pos_x = position_OpenCV.x
    color_pos_y = position_OpenCV.y


    #Get distance from rangefinder
    rangefinder_dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()

    rospy.loginfo("OpenCV Point Position: [ %f, %f, %f ]"%(position_OpenCV.x, position_OpenCV.y, position_OpenCV.z))
    rospy.loginfo("Rangefinder Distance: [ %f]"%(rangefinder_dist))


    #Initialize PoseStamped() message as position of end-effector, with grippers pointed downwards
    pointx = pose_ee[0,0]
    pointy = pose_ee[1,0]
    pointz = pose_ee[2,0]
    quatx = 0
    quaty = 1
    quatz = 0
    quatw = 0



    #Rangefinder distance is directly over object
    if rangefinder_dist < 100:

        #Close Baxter's left gripper
        baxterleft = baxter_interface.Gripper('left')
        baxterleft.close()


    #Rangefinder distance is over object
    if rangefinder_dist < 150:

        incremental_distance = 0.0025

        #X-position of point not within range of center of frame
        if fabs(color_pos_x - 37) > 20:
            if color_pos_x < 37:
                pointy = pose_ee[1,0] + incremental_distance
            else:
                pointy = pose_ee[1,0] - incremental_distance

        #Y-position of point not within range of center of frame
        if fabs(color_pos_y - 94) > 20:
            if color_pos_y < 94:
                pointx = pose_ee[0,0] - incremental_distance
            else:
                pointx = pose_ee[0,0] + incremental_distance

        #X-position and Y-position of point are within range of center of frame
        if fabs(color_pos_x - 37) <= 20 and fabs(color_pos_y - 94) <= 20:
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0] - 0.05


    #Rangefinder distance is nearly over object
    elif rangefinder_dist < 200:
    
        incremental_distance = 0.005

        #X-position of point not within range of center of frame
        if fabs(color_pos_x - 37) > 20:
            if color_pos_x < 37:
                pointy = pose_ee[1,0] + incremental_distance
            else:
                pointy = pose_ee[1,0] - incremental_distance

        #Y-position of point not within range of center of frame
        if fabs(color_pos_y - 94) > 20:
            if color_pos_y < 94:
                pointx = pose_ee[0,0] - incremental_distance
            else:
                pointx = pose_ee[0,0] + incremental_distance

        #X-position and Y-position of point are within range of center of frame
        if fabs(color_pos_x - 37) <= 20 and fabs(color_pos_y - 94) <= 20:
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0] - 0.05


    #Rangefinder distance is barely over object
    elif rangefinder_dist < 250:
 
        incremental_distance = 0.01

        #X-position of point not within range of center of frame
        if fabs(color_pos_x - 31) > 20:
            if color_pos_x < 31:
                pointy = pose_ee[1,0] + incremental_distance
            else:
                pointy = pose_ee[1,0] - incremental_distance

        #Y-position of point not within range of center of frame
        if fabs(color_pos_y - 52) > 20:
            if color_pos_y < 52:
                pointx = pose_ee[0,0] - incremental_distance
            else:
                pointx = pose_ee[0,0] + incremental_distance

        #X-position and Y-position of point are within range of center of frame
        if fabs(color_pos_x - 31) <= 20 and fabs(color_pos_y - 52) <= 20:
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0] - 0.05


    #Rangefinder distance is over object
    elif rangefinder_dist < 350:
 
        incremental_distance = 0.015

        #X-position of point not within range of center of frame
        if fabs(color_pos_x - 16) > 20:
            if color_pos_x < 16:
                pointy = pose_ee[1,0] + incremental_distance
            else:
                pointy = pose_ee[1,0] - incremental_distance

        #Y-position of point not within range of center of frame
        if fabs(color_pos_y - 42) > 20:
            if color_pos_y < 42:
                pointx = pose_ee[0,0] - incremental_distance
            else:
                pointx = pose_ee[0,0] + incremental_distance

        #X-position and Y-position of point are within range of center of frame
        if fabs(color_pos_x - 16) <= 20 and fabs(color_pos_y - 42) <= 20:
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0] - 0.1

        print pointx,pointy


    #Rangefinder distance is too far above object to get an actual value
    else:
        
        incremental_distance = 0.02

        #X-position of point not within range of center of frame
        if fabs(color_pos_x - 0) > 50:
            if color_pos_x < 0:
                pointy = pose_ee[1,0] + incremental_distance
            else:
                pointy = pose_ee[1,0] - incremental_distance

        #Y-position of point not within range of center of frame
        if fabs(color_pos_y - 0) > 50:
            if color_pos_y < 0:
                pointx = pose_ee[0,0] - incremental_distance
            else:
                pointx = pose_ee[0,0] + incremental_distance

        pointz = pose_ee[2,0] - 0.1

        #X-position and Y-position of point are within range of center of frame
        if fabs(color_pos_x - 0) <= 50 and fabs(color_pos_y - 0) <= 50:
            pointx = pose_ee[0,0]
            pointy = pose_ee[1,0]
            pointz = pose_ee[2,0] - 0.15



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

    
    #Send PoseStamped() message to Baxter's movement function
    pub.publish(move_to_pose)


    return 




def main():
    rospy.init_node('target_pose_listener',anonymous = True)


    #Publish temporary color for object detection until node for it is created
    pub_color.publish("Red")


    #Subscribe to Baxter's left endpoint state and Visp autotracker messages
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getPoseEE)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,getPoseTag)
    rospy.Subscriber("/opencv/center_of_object",Point,getPositionObjectfromOpenCV)


    #Wait for left gripper's end effector pose
    while not first_flag:
        pass


    #Continously running to send PoseStamped() message to Baxter
    while not rospy.is_shutdown():
        NewPoseUsingOpenCV(position_object_opencv)

   
    rospy.spin()



if __name__ == '__main__':
    main()