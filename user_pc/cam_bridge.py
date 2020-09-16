# import the necessary packages
from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
from threading import Thread
import numpy as np
import playsound
import argparse
import imutils
import time
import dlib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes
import time, os, sys
from cv_bridge import CvBridge, CvBridgeError

# Instantiate CvBridge
bridge = CvBridge()
angle = 180
scale = 1.0

def callback(data):
	#rospy.loginfo('Data acquiring')
	try:
			cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
			print "e"
	frame = cv_image
	detection(frame)
	
def detection(frame):

	frame = imutils.resize(frame, width=720)	   #450)
	#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)	   #HSV
	#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #B/N

	# get image height, width
	(h, w) = frame.shape[:2
			    ]
	#h=480
	#w=640
	# calculate the center of the image
	center = (w / 2, h / 2) 

	# 180 degrees
	#rotated = imutils.rotate(frame, angle)	#Uncomment for library

	M = cv2.getRotationMatrix2D(center, angle, scale)
	rotated = cv2.warpAffine(frame, M, (w, h))	 #Rotate image
	canny = cv2.Canny(rotated, 1, 10)		 #Detect contours  #PROBAR VALORES
	#(contornos, jerarquia) = cv2.findContours(canny, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
	#cv2.drawContours (rotated, lista_contornos, numero_contornos, color_RGB, 2)

	
	# show the frame
	cv2.imshow("Main camera", rotated)
	#cv2.imshow("Contours", canny)
	key = cv2.waitKey(1) & 0xFF

def ros_listener():
	rospy.init_node('webcam_capture', anonymous=True)
	rospy.Subscriber('/usb_cam/image_raw', Image, callback)
	#rospy.Subscriber('/camera/rgb/image_color', Image, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	print("[INFO] starting video stream thread...")
	time.sleep(1.0)

	# rospy
	ros_listener()
