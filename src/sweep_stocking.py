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
from std_msgs.msg import (
	Header,
	String
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import math

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

import ar_track_alvar_msgs

from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker

pub_color = rospy.Publisher('color_identifier',String)
pub_pose = rospy.Publisher('scanned_stocking_pose',PoseStamped)

def baxter_sweep_stocking(msg):

	)


def tags_listener():
    rospy.init_node('tags_listener',anonymous = True)
	rospy.Subscriber("/visp_auto_tracker/object_position",PoseStamped, baxter_sweep_stocking)
    print "I subscribed to visp auto tracker- now show me a QR code"
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers,getposeEE)
    print "I subscribed to visp ar pose- now show me a tag"
    rospy.spin()

if __name__ == '__main__':
    tags_listener()