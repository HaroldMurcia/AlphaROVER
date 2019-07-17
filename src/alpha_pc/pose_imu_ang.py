import rospy
import time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import tf
from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry


class NodoPos(object): # Crea clase

    def __init__(self):
	self.ekf_pub = rospy.Publisher('/ekf', Odometry, queue_size=10)
        self.enc_start_flag = 0    #una variable con memoria de theta
        self.enc_x=0
	self.enc_y=0
        self.enc_yaw=0
	self.enc_N=100.0
	self.enc_angle_l=0
	self.enc_angle_l_1=0
	self.enc_angle_r=0
	self.enc_angle_r_1=0
	self.enc_inc_r =0
	self.enc_inc_l=0
	self.enc_radio = 7.2/100.0
	self.enc_yo = 0.6
	self.enc_PPR = 6533.0
	self.enc_wr =0
	self.enc_wl =0
	self.enc_vr=0
	self.enc_vl=0
	self.enc_VX = 0
	self.enc_delta_z = 0
        self.enc_Ts = 1/10.0
	self.enc_X=[]
	self.enc_Y=[]
	self.enc_acum_yaw = []
	self.enc_samples=0
	# I M U
	self.imu_samples = 0
	self.imu_Ts = 1/100.0
	self.imu_ax = 0
	self.imu_ay = 0
	self.imu_az = 0
	self.imu_vx = 0
	self.imu_vy = 0
	self.imu_vz = 0
	self.imu_x = 0
	self.imu_y = 0
	self.imu_z = 0
	self.imu_mean_ax = -0.0628382861614
	self.imu_mean_ay = -0.347958862782
	self.imu_mean_az = 9.80008125305
	self.imu_std_ax = 0
        self.imu_std_ay = 0
        self.imu_std_az = 0
	self.imu_yaw= 0
	self.imu_pitch = 0
	self.imu_roll = 0
	self.imu_acum_X = []
	self.imu_acum_Y = []
	self.imu_acum_Z = []
	self.imu_acum_yaw = []
        #rospy.loginfo("Starting node")
        rospy.Subscriber('/encoders', String, self.calcEnc) #10 Hz
	rospy.Subscriber('/imu_data', Imu, self.calcImu)    #100 Hz
        rospy.spin()

    def calcEnc(self,data):
	self.enc_samples+=1
	data=str(data)
	data = data.split(",")
	angle_l = data[0].split(":")
	self.enc_angle_l = float(angle_l[2])
	angle_r = data[1].split(":")
	angle_r = angle_r[1].strip('"')

	self.enc_angle_r = float(angle_r)
	self.enc_inc_l = self.enc_angle_l-self.enc_angle_l_1
	self.enc_inc_r = self.enc_angle_r-self.enc_angle_r_1
	self.enc_angle_l_1 = self.enc_angle_l
	self.enc_angle_r_1 = self.enc_angle_r

	self.enc_wl = (2.0*np.pi/self.enc_PPR)*(self.enc_inc_l/self.enc_Ts)
	self.enc_wr = (2.0*np.pi/self.enc_PPR)*(self.enc_inc_r/self.enc_Ts)
	self.enc_vr = self.enc_radio*self.enc_wr
	self.enc_vl = self.enc_radio*self.enc_wl
	self.enc_delta_z = (self.enc_vr - self.enc_vl)/(2.0*self.enc_yo)

	if self.enc_start_flag == 0:
		self.enc_start_flag = 1
	else:
		self.enc_VX = (self.enc_vr+self.enc_vl)/2.0
		self.enc_yaw = self.enc_yaw + self.enc_delta_z*self.enc_Ts
		self.enc_acum_yaw.append(self.enc_yaw)
 		#print str(self.enc_VX)+" "+str(self.enc_yaw)
		self.enc_yaw = self.imu_yaw
		self.enc_x=self.enc_x+np.cos(self.enc_yaw)*self.enc_VX*self.enc_Ts #-np.cos...
		self.enc_y=self.enc_y+np.sin(self.enc_yaw)*self.enc_VX*self.enc_Ts
		self.enc_X.append( self.enc_x)
		self.enc_Y.append( self.enc_y)
		
		self.publish_odom(self.enc_x, self.enc_y, self.enc_yaw, self.enc_VX, self.enc_delta_z)

    def calcImu(self,data):
	self.imu_samples +=1
    	qx = data.orientation.x
	qy = data.orientation.y
	qz = data.orientation.z
	qw = data.orientation.w
	q = np.array([qx,qy,qz,qw])
	euler = tf.transformations.euler_from_quaternion(q)
	self.imu_yaw = euler[2]
	self.imu_ax = data.linear_acceleration.x - self.imu_mean_ax
	self.imu_ay = data.linear_acceleration.y - self.imu_mean_ay
	self.imu_az = data.linear_acceleration.z - self.imu_mean_az
	R = np.matrix([ [1,0,0,0], [0,np.cos(self.imu_yaw),-np.sin(self.imu_yaw),0], [0,np.sin(self.imu_yaw),np.cos(self.imu_yaw),0],[0,0,0,1]])
	Ain = np.matrix([[-self.imu_ay],[self.imu_ax],[self.imu_az],[1]])
	Aout= np.matmul(R,Ain)
	self.imu_ax = Aout[0]
	self.imu_ay = Aout[1]
	self.imu_az = Aout[2]
	if self.imu_samples <3000:
		self.imu_mean_ax += data.linear_acceleration.x
		self.imu_mean_ay += data.linear_acceleration.y
		self.imu_mean_az += data.linear_acceleration.z
	if self.imu_samples == 3000:
		self.imu_mean_ax = self.imu_mean_ax/3000.0
		self.imu_mean_ay = self.imu_mean_ay/3000.0
		self.imu_mean_az = self.imu_mean_az/3000.0
	if self.imu_samples >3000:
		self.imu_vx = self.imu_vx + self.imu_ax*self.imu_Ts
		self.imu_vy = self.imu_vy + self.imu_ay*self.imu_Ts
		self.imu_vz = self.imu_vz + self.imu_az*self.imu_Ts
		self.imu_x  = self.imu_x  + self.imu_vx*self.imu_Ts
		self.imu_y  = self.imu_y  + self.imu_vy*self.imu_Ts
		self.imu_z  = self.imu_z  + self.imu_vz*self.imu_Ts
		self.imu_acum_X.append(self.imu_x)
		self.imu_acum_Y.append(self.imu_y)
		self.imu_acum_Z.append(self.imu_z)
		self.imu_acum_yaw.append(self.imu_yaw)
		
    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

	self.ekf_pub.publish(odom)
	
    def main(self):
    	#ekf_pub = rospy.Publisher('/ekf', Odometry, queue_size=10)
    	#rospy.init_node("ekf_node")
    	rate = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
       		hello_str = "hello world %s" % rospy.get_time()
        	#rospy.loginfo(hello_str)
        	#pub.publish(hello_str)
        	rate.sleep()
	

if __name__ == '__main__':
	try:
		
		rospy.init_node("ekf_node")
		NodoPos()
	except rospy.ROSInterruptException:
	        pass
	print "Exiting EKF"
