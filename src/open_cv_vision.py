#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import baxter_interface

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError


#Initialize HSV thresholding values
low_h = None
high_h = None
low_s = None
high_s = None
low_v = None
high_v = None

#Create publisher to publish center of object detected
pub = rospy.Publisher('opencv/center_of_object', Point)


#Initialize flags to false
first_flag = False




#Selects HSV thresholding values based on color to look for
def color_selection_hsv(message):

	global low_h,high_h,low_s,high_s,low_v,high_v

	if message.data == "red":
		low_h  = 0
		high_h = 4
		low_s  = 135
		high_s = 245
		low_v  = 60
		high_v = 255

	elif message.data == "green":
		low_h  = 60
		high_h = 90
		low_s  = 85
		high_s = 175
		low_v  = 0
		high_v = 255

	elif message.data == "blue":
		low_h  = 105
		high_h = 115
		low_s  = 135
		high_s = 160
		low_v  = 20
		high_v = 60

	elif message.data == "yellow":
		low_h  = 15
		high_h = 70
		low_s  = 95
		high_s = 255
		low_v  = 68
		high_v = 255

	elif message.data == "orange":
		low_h  = 6
		high_h = 35
		low_s  = 131
		high_s = 255
		low_v  = 0
		high_v = 255


	global first_flag
	first_flag = True




#Thresholds image and stores position of object in (x,y) coordinates of the camera's frame, with origin at center.
def callback(message):


	#Wait for thresholding color to be updated/stored
	while not first_flag:
		pass

	#Capturing image of web camera
	bridge = CvBridge()

	cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
	height, width, depth = cv_image.shape	


	#Converting image to HSV format
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))


	#Morphological opening (remove small objects from the foreground)
	thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
	thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)

	#Morphological closing (fill small holes in the foreground)
	thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
	thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)


	#Finding center of red ball
	ret,thresh = cv2.threshold(thresholded,157,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

	M = cv2.moments(thresh)
	
	A = M['m10']
	B = M['m00']

	if B>500:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])

		P = Point()
		P.x = cx-(width)/2
		P.y = -(cy - (height/2))
		pub.publish(P)


	#Printing to screen the images
	cv2.imshow("Original", cv_image)
	cv2.imshow("Thresholded", thresholded)
	cv2.waitKey(3)



#Subscribes to left hand camera image feed
def main():

	#Create names for OpenCV images and orient them appropriately
	cv2.namedWindow("Original", 1)
	cv2.namedWindow("Thresholded", 2)


	#Initiate node for left hand camera
	rospy.init_node('left_hand_camera', anonymous=True)

	#Subscribe to left hand camera image 
	rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)
	rospy.Subscriber("/color_identifier", String, color_selection_hsv)


	#Keep from exiting until this node is stopped
	rospy.spin()
 
	
         
if __name__ == '__main__':
     main()