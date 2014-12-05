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
    Header,
    String,
    UInt8,
    Float64
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

import ar_track_alvar_msgs

from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker

# So this is how I envision this node playing a role so far... this node will be functioning at the stockings. It will be listening either to the color detection node or tag tracking node. Upon detection of a certain color or tag, it will select the object from the list that it needs to search for and grab using color detection (movement_visp)- in other words, it will send a color to that node in the form of a string

# tag_msg = AlvarMarkers()

pub_color = rospy.Publisher('color_identifier',String)
pub_pose = rospy.Publisher('scanned_stocking_pose',PoseStamped)

m = None

def identify_pres(msg):
    global m
    for m in msg.markers:
        identity = m.id
        home = m.pose
        print "Finding the associated present"
        if identity==1:
            color = "red"
            print "Identified as person 1"
            pub_color.publish(color)
            pub_pose.publish(home)
        elif identity==2:
            color = "blue"
            print "Identified as person 2"
            pub_color.publish(color)
            pub_pose.publish(home)
        elif identity==3:
            color = "green"
            print "Identified as person 3"
            pub_color.publish(color)
            pub_pose.publish(home)
        elif identity==4:
            color = "yellow"
            print "Identified as person 4"
            pub_color.publish(color)
            pub_pose.publish(home)
        else:
            print "Person not identified- will search again"
            baxter_sweep_stocking()
        return

def tag_identity_listener():
    rospy.init_node('tag_identity_listener',anonymous = True)
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers,identify_pres)
    print "I subscribed"
    rospy.spin()

if __name__ == '__main__':
    tag_identity_listener()
