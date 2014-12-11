#!/usr/bin/env python

import argparse
import struct
import sys

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

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState



def getpose(msg):
    print("I GOT IN THE LOOP!!!")
    # global position_new, orientation_new

    # pose = msg.pose
    # position_new = pose.position

    # orientation_new = pose.orientation

    print("msg= ")
    print(msg)



def target_pose_listener():
    print("1")
    rospy.init_node('target_pose_listener',anonymous = True)
    print("2")
    rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,getpose)
    print("3")
    rospy.spin()
    print("4")



if __name__ == '__main__':
    target_pose_listener()