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
pub = rospy.Publisher('color_identifier', String)
pub.publish("Red")


#Setting flags to False
first_flag = False #position end-effector
second_flag = False #QR pose
third_flag = False #opencv position


#Initialization of global variables
tag_msg = PoseStamped()

pose_ee = np.full((7,1), None)

position_object_opencv = Point()

previous_tag = np.zeros((1,3))
print previous_tag[0,1]

movement = 0 #QR code movement (0,1,2)




#Gets pose of end-effector from Baxter
def getPoseEE(msg):

    pose = msg.pose

    position_new = pose.position
    orientation_new = pose.orientation

    global position_ee_x, position_ee_y, position_ee_z, orientation_ee_x, orientation_ee_y, orientation_ee_z, orientation_ee_w, first_flag

    # pose_ee[0:2,:] = position_new
    # pose_ee[3:6,:] = orientation_new

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



#Gets position of object from OpenCV node
def getPositionObjectfromOpenCV(msg):

    global position_object_opencv, third_flag

    position_object_opencv = msg

    third_flag = True

    return


#Creates PoseStamped() message to move appropriately towards object, based on distance away
def NewPoseUsingOpenCV(msg):
 
    
    #Wait for position of object from OpenCV node
    while not third_flag:
        pass
    

    #Store position of object from OpenCV into local variables
    position_OpenCV = msg

    rospy.loginfo("OpenCV Point Position: [ %f, %f, %f ]"%(position_OpenCV.x, position_OpenCV.y, position_OpenCV.z))
   

    color_pos_x = position_OpenCV.x
    color_pos_y = position_OpenCV.y


    rangefinder_dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()

    rospy.loginfo("Rangefinder Distance: [ %f]"%(rangefinder_dist))


    pointx = pose_ee[0,0]
    pointy = pose_ee[1,0]
    pointz = pose_ee[2,0]
    quatx = 0
    quaty = 1
    quatz = 0
    quatw = 0



    #Rangefinder distance is over object
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

    #Returns PoseStamped() message
    return move_to_pose





#Create PoseStamped() message to move Baxter towards QR code
def NewPoseUsingQRcode(msg):

    while not second_flag:
        pass
 
    
    print "position of end-effector=",  pose_ee[0:3,:]

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

    print "Movement counter:",movement
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

        #Close Baxter's left gripper
        baxterleft = baxter_interface.Gripper('left')
        baxterleft.close()


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

    return move_to_pose




#Accepts PoseStamped() message and moves towards it
def BaxterMovement(new_pose):
    rospy.loginfo("ENTERED THE MOVEMENT LOOP")


    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled


    rospy.loginfo("Enabling robot... ") 
    rs.enable()

    #Defining left limb for IK Service Client
    limbw = "left"
    ns = "ExternalTools/" + limbw + "/PositionKinematicsNode/IKService"

    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')    
   
    rospy.loginfo("Received target location message!")


    #Request IK Service Client with new pose to move to
    ikreq.pose_stamp.append(new_pose)
    try:
        #rospy.wait_for_service(ns, 5.0)
        print("in try service")
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        print("in except service")
        return 1
    print resp

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)

    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print "\nIK Joint Solution:\n", limb_joints
        #rospy.loginfo("------------------")
        #print "Response Message:\n", resp
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")


    #Move left limb to limb_joints found from IK
    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(limb_joints)

 
    print("I'm about to sleeeeep")
    rospy.sleep(1)
    print("I just woke up")

    return





def target_pose_listener():
    rospy.init_node('target_pose_listener',anonymous = True)



    #Subscribe to Baxter's left endpoint state and Visp autotracker messages
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getPoseEE)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,getPoseTag)
    rospy.Subscriber("/opencv/center_of_object",Point,getPositionObjectfromOpenCV)



    #Wait for left gripper's end effector pose
    while not first_flag:
        pass


    print "In main loop"
    #Move to home position
    move_to_home_pose = PoseStamped()
    move_to_home_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
    move_to_home_pose.pose.position=Point(
                    x=0.8,
                    y=0.3,
                    z=0.3,
                )
    move_to_home_pose.pose.orientation=Quaternion(
                    x=0,
                    y=1,
                    z=0,
                    w=0,
                )

    BaxterMovement(move_to_home_pose)


    #Calibrate left gripper
    baxterleft = baxter_interface.Gripper('left')
    baxterleft.calibrate()


    #Run main part of loop while rospy is not shutdown
    while not rospy.is_shutdown():

        #If no message is being published from QR code, moves towards object seen from OpenCV node
        while tag_msg.pose.position.x == 0:
            BaxterMovement(NewPoseUsingOpenCV(position_object_opencv))

        #Otherwise, moves toward QR code
        BaxterMovement(NewPoseUsingQRcode(tag_msg))
   
    rospy.spin()



if __name__ == '__main__':
    target_pose_listener()