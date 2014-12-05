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





def main():
    rospy.init_node('baxtermovement',anonymous = True)


    #Subscribe to topic for PoseStamped messages to be sent to
    rospy.Subscriber("/baxter_movement/posestamped",PoseStamped,BaxterMovement)




if __name__ == '__main__':
    main()