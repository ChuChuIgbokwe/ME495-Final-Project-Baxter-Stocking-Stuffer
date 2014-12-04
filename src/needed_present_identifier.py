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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import math

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState


# So this is how I envision this node playing a role so far... this node will be functioning at the stockings. It will be listening either to the color detection node or tag tracking node. Upon detection of a certain color or tag, it will select the object from the list that it needs to search for and grab using color detection (movement_visp)- in other words, it will send a color to that node in the form of a string

def identify_pres(msg):
	if msg == "Person 1":
		color = red
		baxter_sweep_table(color)
		# return color
	else if msg == "Person 2":
		color = blue
		baxter_sweep_table(color)
		# return color
	else if msg == "Person 3":
		color = green
		baxter_sweep_table(color)
		# return color
	else if msg == "Person 4":
		color = yellow
		baxter_sweep_table(color)
		# return color
	else
		print "Person not identified- will search again"
		baxter_sweep_stocking()



def tag_identity_listener:
	rospy.init_node('tag_identity_listener',anonymous = True)
	rospy.Subscriber("/colornodename/colortopicname",String,identify_pres)


if __name__ == '__main__':
    target_pose_listener()
