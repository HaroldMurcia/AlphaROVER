#!/usr/bin/env python
# https://github.com/arebgun/dynamixel_motor 	libreria dynamixel-ROS
# http://docs.ros.org/kinetic/api/dynamixel_msgs/html/msg/JointState.html 	comandos
# rosrun urg_node urg_node _ip_address:="192.168.0.10" _publish_multiecho:="true"

import rospy
from sensor_msgs.msg import Joy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import MultiEchoLaserScan
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import tf
import matplotlib.pyplot as plt
import numpy as np
import sys,math,time,os,datetime

import pandas as pd
import csv
import os

# >0 down
# <0 up

# python move_dynamixel [angle]
# python scan_data 45 

ang_scan = float(sys.argv[1])
dyn_offset = 1.28
count = 0

class First(): # Crea clase
    def __init__(self):
	rospy.init_node("scan_node_mode2")
	rospy.Subscriber("/joy", Joy, self.readJoy)
    	self.pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
	rospy.Subscriber('/tilt_controller/state', JointState, self.read_Dynamixel) # 20Hz
	rospy.Subscriber('/echoes', MultiEchoLaserScan, self.read_LiDAR) #30Hz
	rospy.Subscriber('/ekf', Odometry, self.read_odom)
	rospy.Subscriber('/imu_data', Imu, self.read_imu)
	self.stop_flag = 0
	self.read_pose = 0
	# By Tila
	self.x = 0
	self.y = 0
	self.z = 0
	##
	self.moving= 0
	self.fileText=''
	self.move_ini(ang_scan)
        rospy.spin()

    def createFile(self):
	Ruta=os.getcwd()
    	self.fileText=Ruta+"/data/"+time.strftime("%d-%m-%y")+'-'+time.strftime("%I-%M-%S")+".txt"
    	print("Fichero: "+self.fileText+"\n")
    	f = open(self.fileText,'w')
    	f.close()

    def readJoy(self, data):
	buttons = data.buttons
	axes = data.axes
	if (axes[2]<=-0.9 and buttons[4]==1 and buttons[1]==1):	# LT + LB + B
		self.stop_flag=1
	else:
		self.stop_flag=0

    def read_Dynamixel(self,data):
	#print dir(data)
	self.read_pose = data.current_pos
	self.moving = data.is_moving  
	
    def read_imu(self,data):
	qx = data.orientation.x
    	qy = data.orientation.y
    	qz = data.orientation.z
    	qw = data.orientation.w
    	q = np.array([qx,qy,qz,qw])
    	euler = tf.transformations.euler_from_quaternion(q)
        self.imu_roll = euler[0]
        self.imu_pitch  = euler[1]
        self.imu_yaw   = euler[2]

    def read_odom(self,data):
	self.x = data.pose.pose.position.x
	self.y = data.pose.pose.position.y
	self.z = data.pose.pose.position.z

    def read_LiDAR(self,data):
	#print dir(data)
	ranges = data.ranges
	N=len(ranges)
	intensities = data.intensities
	delta_angle = data.angle_increment
	max_angle =   data.angle_max
	min_angle =   data.angle_min
	theta = np.linspace(min_angle,max_angle,N)
	self.ranges_0 = np.zeros([1,N])
	self.ranges_1 = np.zeros([1,N])
	self.ranges_2 = np.zeros([1,N])
	self.intensities_0 = np.zeros([1,N])
        self.intensities_1 = np.zeros([1,N])
        self.intensities_2 = np.zeros([1,N])
	for k in range(0,N-1):
		aux = str(ranges[k]).split("[")
		aux = str(aux[1]).strip("]")
		aux = aux.split(",")
		aux2 = str(intensities[k]).split("[")
                aux2 = str(aux2[1]).strip("]")
                aux2 = aux2.split(",")

		if len(aux)==1:
			self.ranges_0[0,k] = float(aux[0])
			self.intensities_0[0,k] = (aux2[0])

		elif len(aux)==2:
			self.ranges_0[0,k] = (aux[0])
			self.ranges_1[0,k] = (aux[1])
			self.intensities_0[0,k] = (aux2[0])
			self.intensities_1[0,k] = (aux2[1])

		elif len(aux)==3:
			self.ranges_0[0,k] = (aux[0])
			self.ranges_1[0,k] = (aux[1])
			self.ranges_2[0,k] = (aux[2])
			self.intensities_0[0,k] = (aux2[0])
			self.intensities_1[0,k] = (aux2[1])
			self.intensities_2[0,k] = (aux2[2])


    def move_ini(self,ang):
	if (ang > 60 or ang < -45):
		print "Scan range must be between [-45 60]"
		exit()
	else:
		ang_Rad = ang*math.pi/180.0
		ang_Rad = ang_Rad-dyn_offset
		self.pub.publish(ang_Rad)	
		print "Angle: "+ str(ang_Rad)+" , angle_offset: "+str(dyn_offset)
		while (self.moving == False):
			self.pub.publish(ang_Rad)
		#	print "Waiting for Dynamixel..."
		#	f=1
		#time.sleep(0.2)
		self.ScanTilt(ang_Rad)

    def ScanTilt(self,ang):
	N = 1081
	n = (N*6)+7
	#M = np.zeros((num	, n))
	print "SCANNING..."
	time.sleep(1)
	self.createFile()
	time.sleep(1)
	while (True):
		f = open(self.fileText,"a") #opens file with name of "test.txt"
		time.sleep(0.05)
		dataLine=''
		dataLine=dataLine+str(self.read_pose+dyn_offset)+"\t"
		dataLine=dataLine+str(self.x)+"\t"
		dataLine=dataLine+str(self.y)+"\t"
		dataLine=dataLine+str(self.z)+"\t"
		dataLine=dataLine+str(self.imu_pitch)+"\t"
		dataLine=dataLine+str(self.imu_roll)+"\t"
		dataLine=dataLine+str(self.imu_yaw)+"\t"
		N = self.ranges_0.shape[1]
		for i in range(0,N-1):
			dataLine=dataLine+str(self.ranges_0[0,i])+"\t"
		N = self.ranges_1.shape[1]
		for i in range(0,N-1):
			dataLine=dataLine+str(self.ranges_1[0,i])+"\t"
		N = self.ranges_2.shape[1]
		for i in range(0,N-1):
                        dataLine=dataLine+str(self.ranges_2[0,i])+"\t"
		N = self.intensities_0.shape[1]
                for i in range(0,N-1):
                        dataLine=dataLine+str(self.intensities_0[0,i])+"\t"
		N = self.intensities_1.shape[1]
                for i in range(0,N-1):
                        dataLine=dataLine+str(self.intensities_1[0,i])+"\t"
		N = self.intensities_2.shape[1]
                for i in range(0,N-1):
                        dataLine=dataLine+str(self.intensities_2[0,i])+"\t"
		#M[count,0] = self.read_pose+dyn_offset
		#M[count,1] = self.x
		#M[count,2] = self.y
		#M[count,3] = self.z1
		#M[count,4] = self.imu_pitch
		#M[count,5] = self.imu_roll
		#M[count,6] = self.imu_yaw
		#M[count,7:1087] = self.ranges_0
		#M[count,1088:2168] = self.ranges_1
		#M[count,2169:3249] = self.ranges_2
		#M[count,3250:4330] = self.intensities_0
		#M[count,4331:5411] = self.intensities_1
		#M[count,5412:6492] = self.intensities_2
		dataLine=dataLine+"\n"
		f.write(dataLine)
		if (self.stop_flag==1):
			f.close()
			print "Scan finished"
			exit()

	#self.pub.publish(0.8)
	print "Saving dataFile..."
	exit()
	#self.saveData(M,n)

    def saveData(self,M,n):

	ang = M[:,0]

	df_header=[None]*n

        df_header[0]="Angle"
	dataset = pd.DataFrame({df_header[0]: M[:,0]})

	df_header[1]="X"
	dataset.insert(1, df_header[1], M[:,1], True)

	df_header[2]="Y"
	dataset.insert(2, df_header[2], M[:,2], True)

	df_header[3]="Z"
	dataset.insert(3, df_header[3], M[:,3], True)

	df_header[4]="Pitch"
	dataset.insert(4, df_header[4], M[:,4], True)

	df_header[5]="Roll"
	dataset.insert(5, df_header[5], M[:,5], True)

	df_header[6]="Yaw"
	dataset.insert(6, df_header[6], M[:,6], True)

	N=1080

	for i in range(7,N+7):
		df_header[i]="Range1_"+str(i-7)
		dataset.insert(i, df_header[i], M[:,i], True)

	for i in range(N+7,2*N+7):
		df_header[i]="Range2_"+str(i-N-7)	
		dataset.insert(i, df_header[i], M[:,i], True)

	for i in range(2*N+7,3*N+7):
		df_header[i]="Range3_"+str(i-2*N-7)
		dataset.insert(i, df_header[i], M[:,i], True)

	for i in range(3*N+7,4*N+7):
		df_header[i]="Intensities1_"+str(i-3*N-7)
		dataset.insert(i, df_header[i], M[:,i], True)

	for i in range(4*N+7,5*N+7):
		df_header[i]="Intensities2_"+str(i-4*N-7)
		dataset.insert(i, df_header[i], M[:,i], True)

	for i in range(5*N+7,6*N+7):
		df_header[i]="Intensities3_"+str(i-5*N-7)
		dataset.insert(i, df_header[i], M[:,i], True)
	
	datetime_object = datetime.datetime.now()
	name = "scan_"+str(datetime_object)+".csv"
        print M.shape
	print dataset.shape
	dataset.to_csv(name, sep='\t', encoding='utf-8', index=False)
	
	print "Scan finished"
	os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")	#Lights ON
        time.sleep(0.2)
        os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")	#Lights OFF
	time.sleep(0.2)
	os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")	#Lights ON
        time.sleep(0.2)
        os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")	#Lights OFF
	exit()

if __name__ == '__main__':
    try:
    	cv = First()
    except rospy.ROSInterruptException:
    	print "Interrupted before completion" 
