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
    Float64,
    Bool,
    Int8
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_interface import CHECK_VERSION

from baxter_core_msgs.msg import EndpointState

import ar_track_alvar_msgs

from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker



#Publish to topics the scanned ID and associated color of the present to look for, as well the T/F state of sweep
pub_color = rospy.Publisher('color_identifier',String)
pub_id = rospy.Publisher('scanned_stocking_id',Int8)
pub_statesweep = rospy.Publisher('/start/sweep',Bool, latch=True)
pub_statestockingpose = rospy.Publisher('/start/stockingpose',Bool)

#Create publisher to send PoseStamped() message to topic for Baxter to move towards
pub_baxtermovement = rospy.Publisher('baxter_movement/posestamped', PoseStamped, latch=True)


#Initialization of global variables
completed_list = []
StateSweep =  True
run = True

first_flag = False



#Move to a position where can see all stockings
def MovetoScanRange():

    #Combine position and orientation in a PoseStamped() message
    move_to_pose = PoseStamped()
    move_to_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
    move_to_pose.pose.position=Point(
                    x=0.8,
                    y=0.3,
                    z=0.3,
                )
    move_to_pose.pose.orientation=Quaternion(
                    x=0,
                    y=math.sqrt(2)/2,
                    z=0,
                    w=math.sqrt(2)/2,
                )
    
    #Send PoseStamped() message to Baxter's movement function
    pub_baxtermovement.publish(move_to_pose)

    rospy.sleep(2)

    global first_flag
    first_flag = True

    return



#Listens to ar_pose to identify the stocking and which color the associated present is
def identify_pres(msg):


    # Wait until moved to a position where can see all stockings before starting to scan
    while not first_flag:
        pass

    #Global variables being used
    global StateSweep
    global completed_list


    scanned_identity = []

    # StateSweep = True

    #Onlys runs when told to
    if run == True:

        #Sets the T/F status of sweep to True, such that it continously runs until finds an AR tracker
        StateSweep = True
        color = None

        #Then obtains the markers message of AlvarMarkers
        for m in msg.markers:

            #Creates local variables for the ID of found stocking
            identity = m.id
            
            scanned_identity.append(identity)

        print "Scanned Identities:",scanned_identity

        #Runs while found stocking has not already been filled
        if len(scanned_identity) != 0:

            #Finds first ID not in completed list
            i = len(scanned_identity)-1
            while scanned_identity[i] not in completed_list and i>-1 and scanned_identity[i]>0 and scanned_identity[i]<5:


                identity = scanned_identity[i]
                sent_identity = int(identity)

                #Stocking #1 found
                if identity==1:

                    #Associated color of present is red
                    color = "red"

                    print "Identified as person 1, with colored object", color



                #Stocking #2 found
                elif identity==2:

                    #Associated color of present is blue
                    color = "blue"

                    print "Identified as person 2, with colored object", color

                    # #Adds Stocking #2 to completed list, and turns the status of sweep to F so it does not continue looking
                    # completed_list.append(identity)
                    # StateSweep = False

                    # #Publishes color of present to look for, as well as stocking ID and status of determining stocking pose to T
                    # pub_color.publish(color)
                    # pub_id.publish(sent_identity)
                    # pub_statestockingpose.publish(True)

                #Stocking #3 found
                elif identity==3:

                    #Associated color of present is green
                    color = "green"

                    print "Identified as person 3, with colored object", color

                    # #Adds Stocking #3 to completed list, and turns the status of sweep to F so it does not continue looking
                    # completed_list.append(identity)
                    # StateSweep = False

                    # #Publishes color of present to look for, as well as stocking ID and status of determining stocking pose to T
                    # pub_color.publish(color)
                    # pub_id.publish(sent_identity)
                    # pub_statestockingpose.publish(True)

                #Stocking #4 found
                elif identity==4:

                    #Associated color of present is yellow
                    color = "yellow"

                    print "Identified as person 4, with colored object", color

                    # #Adds Stocking #4 to completed list, and turns the status of sweep to F so it does not continue looking
                    # completed_list.append(identity)
                    # StateSweep = False

                    # #Publishes color of present to look for, as well as stocking ID and status of determining stocking pose to T
                    # pub_color.publish(color)
                    # pub_id.publish(sent_identity)
                    # pub_statestockingpose.publish(True)

                i = i-1



            #Adds first stocking to completed list, and turns the status of sweep to F so it does not continue looking
            completed_list.append(identity)
            StateSweep = False


            #Publishes color of present to look for, as well as stocking ID and status of determining stocking pose to T
            pub_color.publish(color)
            pub_id.publish(sent_identity)
            pub_statestockingpose.publish(True)



        #Publishes the status of sweep to F if a new stocking was found, otherwise it remains T
        pub_statesweep.publish(StateSweep)

        print "Status of StateSweep:",StateSweep

    return




#Obtain T/F state of whether to start looking for a new present
def getStatusSweep(msg):

    global run

    run = msg.data

    return



#Initializes node and subscribers
def tag_identity_listener():

    rospy.init_node('tag_identity_listener',anonymous = True)

    #Subscribe to AlvarMarkers message and T/F status of sweep
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers,identify_pres)
    rospy.Subscriber("/start/sweep",Bool,getStatusSweep)

    #Move to a position where can see all stockings
    MovetoScanRange()

    rospy.sleep(5)

    #Initially start the sweep for the first stocking
    pub_statesweep.publish(True)

    rospy.spin()



if __name__ == '__main__':
    tag_identity_listener()