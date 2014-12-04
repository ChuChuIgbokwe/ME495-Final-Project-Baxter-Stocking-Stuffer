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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import math

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

def baxter_sweep_stocking():