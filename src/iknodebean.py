#!/usr/bin/env python

import argparse
import struct
import sys
import numpy as np
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import math

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

# right = baxter_interface.Limb('right')

# pub = rospy.Publisher('/visp_auto_tracker/object_position',PoseStamped)

global previous


# position_new=EndpointState().pose.position

position_ee_x=0
position_ee_y=0
position_ee_z=0


# orientation_new=EndpointState().pose.orientation

orientation_ee_x=0
orientation_ee_y=0
orientation_ee_z=0
orientation_ee_w=0

previous = np.zeros((1,3))

def getpose(msg):

    pose = msg.pose

    position_new = pose.position
    orientation_new = pose.orientation

    global position_ee_x, position_ee_y, position_ee_z, orientation_ee_x, orientation_ee_y, orientation_ee_z, orientation_ee_w


    position_ee_x=position_new.x
    position_ee_y=position_new.y
    position_ee_z=position_new.z

    orientation_ee_x=orientation_new.x
    orientation_ee_y=orientation_new.y
    orientation_ee_z=orientation_new.z
    orientation_ee_w=orientation_new.w

    rospy.sleep(0.5)

    # return msg
    # rospy.init_node('getpose',anonymous = True)
    # rospy.Subscriber("/robot/limb/left/endpoint_state/",PoseStamped,ik_test)

def ik_test(msg):
    print("ENTERED THE IK_TEST LOOP")
    print("message=", msg)
    # limb = baxter_interface.Limb('right')
    limbw = "left"
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
	
    rs.enable()
    # rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limbw + "/PositionKinematicsNode/IKService"

    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
    # global position_new, orientation_new 
    rospy.loginfo("Received target location message!")
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getpose)
    print("position new=",  position_ee_x, position_ee_y, position_ee_z)

    
    position = msg.pose.position
    quat = msg.pose.orientation
    # print(position_new,orientation_new)
    rospy.loginfo("Old Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))

    xold = position.x
    yold = position.y
    zold = position.z

    xoold = quat.x
    yoold = quat.y
    zoold = quat.z
    woold = quat.w

    global previous

    # if math.fabs(xold - previous[0,0])<1e-1 and math.fabs(yold - previous[0,1])<1e-1 and math.fabs(zold == previous[0,2])<1e-1:
    #     print("Entered IF loop")
    #     xold = 0
    #     yold = 0
    #     zold = 0
    #     xoold = 0
    #     yoold = 0
    #     zoold = 0
    #     woold = 1

    #print("values", xold,yold,zold)

    # position.x = xold + position_new.x
    # position.y = -zold + position_new.y
    # position.z = yold + position_new.z

    # position.x = pointx
    # position.y = pointy
    # position.z = pointz

    pointx = position_ee_x - xold
    pointy = position_ee_y - yold
    pointz = position_ee_z - zold

    # pointx = 0.588876033715
    # pointy = 0.3068756
    # pointz = 0.323650990769

    # position.Point.x = 0.588876033715
    # position.Point.y = 0.3068756
    # position.Point.z = 0.323650990769

    rospy.loginfo("Old Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))  

    # quat.x = quatx
    # quat.y = quaty
    # quat.z = quatz
    # quat.w = quatw

    quatx = orientation_ee_x - xoold 
    quaty = orientation_ee_y - yoold
    quatz = orientation_ee_z - zoold
    quatw = orientation_ee_w# - woold
    print("quat=", quatx,quaty,quatz,quatw)

    # quatx = orientation_new.x
    # quaty = orientation_new.y
    # quatz = orientation_new.z
    # quatw = orientation_new.w

    # quatx = -0.366895
    # quaty = 0.88598
    # quatz = 0.108156
    # quatw = 0.262162

    # quat.Quaternion.x = -0.366895
    # quat.Quaternion.y = 0.88598
    # quat.Quaternion.z = 0.108156
    # quat.Quaternion.w = 0.262162
 
    rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
    rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))   

    pose = PoseStamped()
    pose.header=Header(stamp=rospy.Time.now(), frame_id='base')

    pose.pose.position=Point(
                    x=pointx,
                    y=pointy,
                    z=pointz,
                )
    pose.pose.orientation=Quaternion(
                    x=quatx,
                    y=quaty,
                    z=quatz,
                    w=quatw,
                )

    print("desired position", pose.pose.position)
    print("desired orientation", pose.pose.orientation)
    print("pose",pose)
    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
        print("in try service")
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        print("in except service")
        return 1

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
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    # angles = limb_joints
    # print("limb_joints=")

    limb = baxter_interface.Limb('left')
    limb.move_to_joint_positions(limb_joints)
    # print angles
    # return angles

    # poseold = PoseStamped()
    # poseold.header=Header(stamp=rospy.Time.now(), frame_id='base')

    # poseold.pose.position=Point(
    #                 x=0,
    #                 y=0,
    #                 z=0,
    #             )
    # poseold.pose.orientation=Quaternion(
    #                 x=0,
    #                 y=0,
    #                 z=0,
    #                 w=1,
    #             )

    # pub.publish(poseold)
    previous[0,0] = msg.pose.position.x
    previous[0,1] = msg.pose.position.y
    previous[0,2] = msg.pose.position.z
    print("I'm about to sleeeeep")
    rospy.sleep(2.75)
    print("I just woke up")
    return limb_joints





def target_pose_listener():
    rospy.init_node('target_pose_listener',anonymous = True)
    rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped,ik_test)
    # limb = baxter_interface.Limb('left')
    # limb.move_to_joint_positions(ik_test())    
    rospy.spin()

 #    global position_new
 #    ans=position_new
 #    print("ans= ")
 #    print(ans)
	# 


if __name__ == '__main__':
    target_pose_listener()
    # rospy.init_node('target_pose_listener',anonymous = True)

    # pose = PoseStamped()
    # pose.header=Header(stamp=rospy.Time.now(), frame_id='base')

    # pose.pose.position=Point(
    #                 x=0.656982770038,
    #                 y=-0.852598021641,
    #                 z=0.0388609422173,
    #             )
    # pose.pose.orientation=Quaternion(
    #                 x=0.367048116303,
    #                 y=0.885911751787,
    #                 z=-0.108908281936,
    #                 w=0.261868353356,
    #             )
    # print("printing pose")
    # print(pose)
    print("11111111111111111111111111111")
    print("22222222222222222222222222222")