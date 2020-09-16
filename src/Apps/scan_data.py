#!/usr/bin/env python
# https://github.com/arebgun/dynamixel_motor 	libreria dynamixel-ROS
# http://docs.ros.org/kinetic/api/dynamixel_msgs/html/msg/JointState.html 	comandos
# rosrun urg_node urg_node _ip_address:="192.168.0.10" _publish_multiecho:="true"

import rospy
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

# >0 down
# <0 up

# python move_dynamixel [from] [to] [step]
# python scan_data 45 -45 1

min_tilt = float(sys.argv[1])
max_tilt = float(sys.argv[2])
res_tilt = float(sys.argv[3])
dyn_offset = 1.28
count = 0

class First(): # Crea clase
    def __init__(self):
	rospy.init_node("scan_node")
    	self.pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
	rospy.Subscriber('/tilt_controller/state', JointState, self.read_Dynamixel) # 20Hz
	rospy.Subscriber('/echoes', MultiEchoLaserScan, self.read_LiDAR) #30Hz
	rospy.Subscriber('/ekf', Odometry, self.read_odom)
	rospy.Subscriber('/imu_data', Imu, self.read_imu)
	self.move_ini(min_tilt,max_tilt,res_tilt)
        rospy.spin()

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
	intensities = data.intensities
	delta_angle = data.angle_increment
	max_angle =   data.angle_max
	min_angle =   data.angle_min
	N = len(ranges)-1
	theta = np.linspace(min_angle,max_angle,N)
	self.ranges_0 = np.zeros([1,N])
	self.ranges_1 = np.zeros([1,N])
	self.ranges_2 = np.zeros([1,N])
	self.intensities_0 = np.zeros([1,N])
        self.intensities_1 = np.zeros([1,N])
        self.intensities_2 = np.zeros([1,N])
	
	for k in range(0,N):
		aux = str(ranges[k]).split("[")
		aux = str(aux[1]).strip("]")
		aux = aux.split(",")
		aux2 = str(intensities[k]).split("[")
                aux2 = str(aux2[1]).strip("]")
                aux2 = aux2.split(",")

		if len(aux)==1:
			self.ranges_0[0,k] = float(aux[0])
			self.intensities_0[0,k] = float(aux2[0])

		elif len(aux)==2:
			self.ranges_0[0,k] = float(aux[0])
			self.ranges_1[0,k] = float(aux[1])
			self.intensities_0[0,k] = float(aux2[0])
			self.intensities_1[0,k] = float(aux2[1])

		elif len(aux)==3:
			self.ranges_0[0,k] = float(aux[0])
			self.ranges_1[0,k] = float(aux[1])
			self.ranges_2[0,k] = float(aux[2])
			self.intensities_0[0,k] = float(aux2[0])
			self.intensities_1[0,k] = float(aux2[1])
			self.intensities_2[0,k] = float(aux2[2])


    def move_ini(self,minAng,maxAng,step):
	
	if (minAng>45 or maxAng<-45 or step<0.3):
		print "Scan range must be between [-45 45], with resolution >=0.3"
		exit()
	else:
		minRad = minAng*math.pi/180
		maxRad = maxAng*math.pi/180
		stepRad = step*math.pi/180
	
		minRad = minRad-dyn_offset
		maxRad = maxRad-dyn_offset
		
		print "From: "+ str(minAng)	
		time.sleep(0.5)
		print "To: "+ str(maxAng)	
		time.sleep(0.5)
		print "Step: "+ str(step)	
		time.sleep(0.5)
	
		tAng = minAng-maxAng
		print "Scanned angle: "+ str(tAng)
	
		numScans = int((abs(minAng)+abs(maxAng))/step+1)
		print "Num.Scans: "+ str(numScans)
	
		self.ScanTilt(minRad,maxRad,stepRad,numScans)

    def ScanTilt(self,Min,Max,res,num):
	k = 0
	count = 0
	res = abs(res)
	current_pose = Min
	 
	self.pub.publish(current_pose)

	fMin = 0
	fMin = int(fMin)
	
	fMax = 2*abs(max_tilt)
	fMax = int(fMax)
	
	fres = res_tilt
	fres = int(fres)

	N = 1081
	n = (N*6)+7
	M = np.zeros((num, n))

	time.sleep(0.5)

	while (self.moving == True):
		#print "Waiting for Dynamixel..."
		f=1
	time.sleep(0.2)
	print "SCANNING..."

	for i in range(0,num):
			
		#print self.read_pose+dyn_offset
		#print current_pose
		self.pub.publish(current_pose)
		M[count,0] = self.read_pose+dyn_offset
		current_pose = current_pose-res	
		while (self.moving == True):
			#print "Waiting for Dynamixel..."
			f=1
		time.sleep (0.2)
#		M[count,1] = self.x
#		M[count,2] = self.y
#		M[count,3] = self.z
		M[count,4] = self.imu_pitch
		M[count,5] = self.imu_roll
		M[count,6] = self.imu_yaw
		M[count,7:1087] = self.ranges_0
		M[count,1088:2168] = self.ranges_1
		M[count,2169:3249] = self.ranges_2
		M[count,3250:4330] = self.intensities_0
		M[count,4331:5411] = self.intensities_1
		M[count,5412:6492] = self.intensities_2
		count = count+1
	self.pub.publish(0.8)
	print "Saving dataFile..."
	#exit()
	self.saveData(M,n)

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
