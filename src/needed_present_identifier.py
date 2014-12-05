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

pub = rospy.Publisher('color_identifier',String)
pub2 = rospy.Publisher('number',Float64)

def get_id_tag(msg):
    # global tag_msg
    tag_msg = msg

    if len(tag_msg.markers)== 0:
        tag_msg1 = tag_msg.markers
    else:
        temp = AlvarMarker()

        tag_msg2 = tag_msg

def identify_pres(msg):
    tag_id = tag_msg
    if msg==1:
        color = red
        pub.publish(color)
    elif msg==2:
        color = blue
        pub.publish(color)
    elif msg==3:
        color = green
        pub.publish(color)
    elif msg==4:
        color = yellow
        pub.publish(color)
    else:
        print "Person not identified- will search again"
        baxter_sweep_stocking()

def tag_identity_listener():
    rospy.init_node('tag_identity_listener',anonymous = True)
    # rospy.Subscriber("/ar_pose_marker",AlvarMarkers,get_id_tag)
    rospy.Subscriber("/number",Float64,get_id_tag)
    # temp = Float64()
    # temp = 3
    # pub2.publish(temp)
    print "I subscribed"
    rospy.spin()

if __name__ == '__main__':
    tag_identity_listener()
