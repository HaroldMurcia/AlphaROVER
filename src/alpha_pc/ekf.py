# Position estimation
# @author: harold.murcia
# Marzo 16 /19

import rospy
import time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import matplotlib.pyplot as plt
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater


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
        self.enco_inc_l=0
        self.enc_radio = 7.2/100.0
        self.enc_yo = 0.72
        self.enc_PPR = 6533.0
        self.enc_wr =0
        self.enc_wl =0
	self.theta_l=0
	self.theta_r=0
        self.enc_vr=0
        self.enc_vl=0
        self.enc_delta_s = 0
	self.enc_delta_s_1 =0
        self.enc_delta_yaw = 0
        self.enc_Ts = 1/10.0
	self.enc_tic = 0
	self.enc_toc = 1/10.0
        self.enc_X=[]
        self.enc_Y=[]
        self.enc_acum_yaw = []
        self.enc_samples=0
        self.enc_WL = []
        self.enc_WR = []
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
        self.imu_std_yaw = []
        self.imu_std_ax = 0
        self.imu_std_ay = 0
        self.imu_std_az = 0
        self.imu_mean_yaw = []
        self.imu_yaw = 0
        self.imu_pitch = 0
        self.imu_roll  = 0
        self.imu_acum_X = []
        self.imu_acum_Y = []
        self.imu_acum_Z = []
        self.imu_acum_yaw = []
	self.mag_yaw = 0
	self.mag_acum_yaw = []
	self.mag_integral_yaw = 0
	self.mag_aux = []
        #
        self.enc_integral_yaw = 0
        self.enc_aux = []
        self.kalman_no_filtered_yaw = []
        # K A L M A N
        self.kalman_P = 1.0*np.identity(3)
        self.kalman_Q = 1.0*np.matrix([[1.0, 0, 0],[0, 1.0, 0],[0,0,0.1]])
	self.kalman_G = np.matrix([[0, 0],[0,0], [0,0]])
        self.kalman_R = 0.0265*1.0
        self.kalman_Xk = np.matrix([[0],[0],[100.0]])
        self.kalman_filtered_yaw = []
        # INIT
        #rospy.loginfo("Starting node")
        rospy.Subscriber('/encoders', String, self.Enc_Kalman) #10 Hz
        rospy.Subscriber('/imu_data', Imu, self.calIMU)    #100 Hz
	rospy.Subscriber('/mti/sensor/magnetic',Vector3Stamped, self.magIMU)    #100 Hz
        rospy.spin()

    def Enc_Kalman(self,data):
	self.enc_toc= rospy.get_time()
	self.enc_Ts = (self.enc_toc - self.enc_tic)
	self.enc_tic = rospy.get_time()
        self.enc_samples+=1
        data=str(data)
        data = data.split(",")
        angle_l = data[0].split(":")
        self.enc_angle_l = float(angle_l[2])
        angle_r = data[1].split(":")
        angle_r = angle_r[1].strip('"')
        self.enc_angle_r = float(angle_r)
	if self.enc_samples == 1:
		self.enc_angle_l_1 = self.enc_angle_l
		self.enc_angle_l_1 = self.enc_angle_l
        self.enc_inc_l = self.enc_angle_l-self.enc_angle_l_1
        self.enc_inc_r = self.enc_angle_r-self.enc_angle_r_1
        self.enc_angle_l_1 = self.enc_angle_l
        self.enc_angle_r_1 = self.enc_angle_r
        self.enc_wl = (2.0*np.pi/self.enc_PPR)*(self.enc_inc_l/self.enc_Ts)
        self.enc_wr = (2.0*np.pi/self.enc_PPR)*(self.enc_inc_r/self.enc_Ts)
	self.enc_vr = self.enc_radio*self.enc_wr
	self.enc_vl = self.enc_radio*self.enc_wl
	self.enc_VX = (self.enc_vr+self.enc_vl)/2.0
	self.theta_l = self.theta_l + (2.0*np.pi/self.enc_PPR)*self.enc_inc_l
	self.theta_r = self.theta_r + (2.0*np.pi/self.enc_PPR)*self.enc_inc_r
        A = np.matrix([[self.enc_radio, self.enc_radio],[self.enc_radio/self.enc_yo, -self.enc_radio/self.enc_yo]])
        U = np.matrix([[self.enc_wr],[self.enc_wl]])
        X = 0.5*A*U
	self.enc_delta_s_1 = self.enc_delta_s
        self.enc_delta_s = X[0]*self.enc_Ts
        self.enc_delta_yaw = X[1]*self.enc_Ts
	
        if self.enc_samples < 50 and self.enc_samples>10: #:520 and self.enc_samples>100    
	    self.imu_acum_yaw.append(self.imu_yaw)
	    self.mag_acum_yaw.append(self.mag_yaw)
            #rospy.loginfo("Samples " + str(self.enc_samples))
        if self.enc_samples > 50: # 520:
            self.enc_WR.append(self.enc_wr)
            self.enc_WL.append(self.enc_wl)
            media = np.array( self.imu_acum_yaw)
	    media_mag = np.array(self.mag_acum_yaw)
            self.kalman_no_filtered_yaw.append( float(self.imu_yaw-media.mean()) )
            self.enc_integral_yaw += self.enc_delta_yaw
	    #self.mag_integral_yaw += self.mag_yaw
	    self.mag_aux.append(float( self.mag_yaw -media_mag.mean() ))
            self.enc_aux.append(float( self.enc_integral_yaw ))
	    # K A L M A N F i l t e r
            # Model update
            kalman_delta_x   = float( self.enc_delta_s*( np.cos( self.kalman_Xk[2] +self.enc_delta_yaw/2.0 )*np.cos(self.imu_pitch) ))
            kalman_delta_y   = float( self.enc_delta_s*( np.sin( self.kalman_Xk[2] +self.enc_delta_yaw/2.0 )*np.cos(self.imu_pitch) ))
            kalman_delta_yaw = float( self.enc_delta_yaw*( np.cos(self.imu_roll)/(np.cos(self.imu_pitch)+1e-6) ))
            #print "delta: "+ str(np.matrix([[kalman_delta_x],[kalman_delta_y],[kalman_delta_yaw]]))
	    self.kalman_Xk   = self.kalman_Xk + np.matrix([[kalman_delta_x],[kalman_delta_y],[kalman_delta_yaw]])
            #self.kalman_filtered_yaw.append( float( self.kalman_Xk[2]) )
            # Jacobian computation
            kalman_F = np.matrix([[1.0, 0.0, -self.enc_delta_s*np.sin( self.kalman_Xk[2] +self.enc_delta_yaw/2.0)*np.cos(self.imu_pitch)], [0.0, 1.0, self.enc_delta_s*np.cos( self.kalman_Xk[2] +self.enc_delta_yaw/2.0)*np.cos(self.imu_pitch)], [0.0, 0.0, 1.0*np.cos(self.imu_roll)/(np.cos(self.imu_pitch)+1e-6)]])
            kalman_H = np.matrix([0.0,0.0,1.0])
            # Error covariance
	    #self.kalman_G = np.matrix([[self.enc_Ts*np.cos( self.kalman_Xk[2] +self.enc_delta_yaw/2.0 ), 0],[self.enc_Ts*np.sin( self.kalman_Xk[2] +self.enc_delta_yaw/2.0 ),0], [0,self.enc_Ts]])
	    self.kalman_P =  np.dot(np.dot(kalman_F, self.kalman_P), kalman_F.T) +self.kalman_Q
            # Kalman filter Gain update
            kalman_S   =  np.linalg.inv( self.kalman_R + np.dot(  np.dot(kalman_H,self.kalman_P) ,kalman_H.T) )
            kalman_K   =  np.dot( np.dot(self.kalman_P,kalman_H.T) , kalman_S)
            # Kalman measurement update
	    kalman_Yk  = np.dot(kalman_H, self.kalman_Xk)
            kalman_Zk  = float( self.imu_yaw - media.mean() )
            self.kalman_Xk = self.kalman_Xk + np.dot(kalman_K, (kalman_Zk-kalman_Yk))
            #print str("KalmanK add ")+str(np.dot(kalman_K, (kalman_Zk-kalman_Yk)))
	    #print "Z: "+str(kalman_Zk)+"\t"+"Y: "+str(kalman_Yk)+"\t"+"KalmanK: "+str(kalman_K)
            self.kalman_P = np.dot( (np.identity(3) - np.dot(kalman_K,kalman_H)) ,self.kalman_P)
	    #self.enc_X.append( float( self.kalman_Xk[0]) )
	    #self.enc_Y.append( float( self.kalman_Xk[1]) )
	    self.enc_x = float( self.kalman_Xk[0])
	    self.enc_y = float( self.kalman_Xk[1])
	    self.kalman_filtered_yaw = float( self.kalman_Xk[2])

	    self.publish_odom(self.enc_x, self.enc_y, self.kalman_filtered_yaw, self.enc_VX, self.enc_delta_yaw)
	    
    def magIMU(self,data):
    	magx= np.array(data.vector.x)
    	magy= np.array(data.vector.y)
    	magz= np.array(data.vector.z)
        mag_norm = np.sqrt(np.power(magx,2)+np.power(magy,2)+np.power(magz,2))
        magx = magx / mag_norm
        magy = magy / mag_norm
        magz = magz / mag_norm
        Yh = (magy * np.cos(self.imu_roll)) - (magz * np.sin(self.imu_roll))
        Xh = (magx * np.cos(self.imu_pitch))+(magy * np.sin(self.imu_roll)*np.sin(self.imu_pitch)) + (magz * np.cos(self.imu_roll) * np.sin(self.imu_pitch))
    	#mag_yaw = np.arctan(np.array(magy),np.array(magx))
        mag_yaw = np.arctan(-np.array(Yh), np.array(Xh))
        self.mag_yaw = mag_yaw
        #print str(mag_yaw)

    def calIMU(self,data):
    	self.imu_samples +=1
        qx = data.orientation.x
    	qy = data.orientation.y
    	qz = data.orientation.z
    	qw = data.orientation.w
    	q = np.array([qx,qy,qz,qw])
    	euler = tf.transformations.euler_from_quaternion(q)
        self.imu_pitch = euler[0]
        self.imu_roll  = euler[1]
        self.imu_yaw   = euler[2]
	#print str(euler[2])
	#print str(self.imu_pitch) + "\t" + str(self.imu_roll)+"\t"+str(self.imu_yaw)

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
    	pub = rospy.Publisher('chatter', String, queue_size=10)
    	rospy.init_node('talker', anonymous=True)
    	rate = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
       		hello_str = "hello world %s" % rospy.get_time()
        	#rospy.loginfo(hello_str)
        	pub.publish(hello_str)
        	rate.sleep()


if __name__ == '__main__':
    rospy.init_node("ekf_node")
    cv = NodoPos()
