#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  1 22:27:15 2020

__author__ = sebastian.tilaguy@gmail.com
__version__ = "1.0"
__maintainer__ = "Sebastian Tilaguy"
__email__ = "sebastian.tilaguy@gmail.com"
__status__ = "Development"
"""

import rospy
import rosnode
import math
import numpy as np
import os
from std_msgs.msg import Float32, Int64
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

def quaternion2euler(q):
    ### q = {q.x, q.y, q.z, q.w}
    # roll
    sq = 2.0 * (q.z * q.x + q.y * q.w)
    cq = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll =  math.atan2(sq, cq)
    # pitch
    sq = 2.0 * (q.w * q.y - q.z * q.x)
    if np.abs(sq) >=1:
        pitch = math.copysign(np.pi/2.0, sq) # use 90 degrees if out of range
    else:
        pitch = math.asin(sq)
    # yaw
    sq = 2.0 * (q.w * q.z + q.x * q.y)
    cq = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw =  math.atan2(sq, cq)
    return (roll,pitch,yaw)

def euler2quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x,y,z,w)

class NodoPos(object):
    #%% Inizialization
    def __init__(self):
        self.ekf_pub = rospy.Publisher('/ekf2_5', Odometry, queue_size=10)
        self.Wl_filt = rospy.Publisher('/ekf/Wl', Float32, queue_size=10)
        self.Wl_filt = rospy.Publisher('/ekf/Wr', Float32, queue_size=10)
        #%% Encoder variables
        self.enc_Ts = 0.0
        self.enc_tic =  0.0
        self.wr = 0.0
        self.wl = 0.0
        self.wr_1 = 0.0
        self.wl_1 = 0.0
        self.wr_2 = 0.0
        self.wl_2 = 0.0
        #%% Filter_encoders variables
        self.acce = 20.0
        self.wr_f = 0.0
        self.wl_f = 0.0
        self.wr_f_2 = 0.0
        self.wr_f_1 = 0.0
        self.wl_f_2 = 0.0
        self.wl_f_1= 0.0
        #%% IMU variables
        self.flag_imu = True
        self.euler1 = [0,0,0]
        self.imu_toc = 0.0
        self.imu_Ts = 0.0
        self.imu_tic = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        self.Acc_x = 0
        self.Acc_y = 0
        self.Acc_z = 0
        self.Acc_x0 = 0
        self.Acc_y0 = 0
        self.Acc_z0 = 0
        #%% GPS variables
        self.flag_gps = True
        self.gps_0x = 0
        self.gps_0y = 0
        self.gps_x = 0
        self.gps_y = 0
        self.gps_x1 = 0
        self.gps_y1 = 0
        self.gps_x2 = 0
        self.gps_y2 = 0
        self.gps_fx = 0
        self.gps_fy = 0
        self.gps_fx1 = 0
        self.gps_fy1 = 0
        self.gps_fx2 = 0
        self.gps_fy2 = 0
        self.d_1 = 0.0
        self.d = 0.0
        
        #%% Mag variables
        self.flag_mag = True
        self.yaw_mag1 = 0.0
        self.mag_toc = 0.0
        self.mag_Ts = 0.0
        self.mag_tic = 0.0
        self.mag_yaw = 0.0
        self.mag_yaw_1 = 0.0
        self.Wz = 0.0
        self.x_mag = 0.0
        self.y_mag = 0.0
        self.z_mag = 0.0
        self.Wz_2 = 0.0
        self.Wz_1 = 0.0
        self.Wz_f2 = 0.0
        self.Wz_f1 = 0.0
        
        #%% Skid Steer model
        self.x_model = np.array([[0],       # x_N
                                 [0],       # y_E
                                 [0],       # yaw
                                 [5.0],     # y_ICR_r
                                 [-5.0],    # y_ICR_l
                                 [5.0],     # x_ICR_v
                                 [0],       # vr
                                 [0]])      # vl
        self.P_ICR = 100*np.identity(8)
        # Covariance matrix of measurement noise
        a = np.power(0.001*np.pi/180,2)
        # self.R_model = np.array([[2.5**2, 0, 0, 0, 0],   # [m] - GPS
        #                          [0, 2.5**2, 0, 0, 0],   # [m] - GPS
        #                          [0, 0, a, 0, 0],        # [rad] - Mag
        #                          [0, 0, 0, 0.02**2, 0], # [m/s] - Enc R
        #                          [0, 0, 0, 0, 0.02**2]])# [m/s] - Enc L
        self.R_model = np.array([[2.5**2, 0, 0, 0],   # [m] - GPS
                                 [0, 2.5**2, 0, 0],   # [m] - GPS
                                 [0, 0, 0.02**2, 0], # [m/s] - Enc R
                                 [0, 0, 0, 0.02**2]])# [m/s] - Enc L
        # Covariance matrix of process noise
        self.Q_model = np.zeros([8,8])
        self.Q_model[0,0] = 0.0005**2                  # [m]
        self.Q_model[1,1] = 0.0005**2                  # [m]
        self.Q_model[2,2] = np.power(12*np.pi/180,2) # [rad]
        self.Q_model[3,3] = 0.2**2                 # [m]
        self.Q_model[4,4] = 0.2**2                 # [m]
        self.Q_model[5,5] = 0.2**2                 # [m]
        self.Q_model[6,6] = 0.2**2                 # [m/s]
        self.Q_model[7,7] = 0.2**2                 # [m/s]
        
        #%% Kalman variables
        # Imu model
        self.A_imu = np.identity(7)
        self.B_imu = np.zeros([7,3])
        self.x_imu = np.zeros([7,1]) # [x y yaw vx vy Bx By]'
        # self.C_imu = np.array([[1,0,0,0,0,0,0],
        #                        [0,1,0,0,0,0,0],
        #                        [0,0,1,0,0,0,0],
        #                        [1,0,0,0,0,0,0],
        #                        [0,1,0,0,0,0,0],
        #                        [0,0,1,0,0,0,0]])
        self.C_imu = np.array([[1,0,0,0,0,0,0],
                               [0,1,0,0,0,0,0],
                               [0,0,1,0,0,0,0],
                               [0,0,1,0,0,0,0]])
        # Matrix error of covariance
        self.kalman_P = 100*np.identity(7)
        # Covariance matrix of process noise
        self.kalman_Q = np.zeros([7,7])
        self.kalman_Q[0,0] = 0.01**2
        self.kalman_Q[1,1] = 0.01**2
        self.kalman_Q[2,2] = np.power(0.5*np.pi/180,2)
        self.kalman_Q[3,3] = 4**2
        self.kalman_Q[4,4] = 8**2
        self.kalman_Q[5,5] = 0.001
        self.kalman_Q[5,5] = 0.001
        # Covariance matrix of measurement noise
        self.kalman_R = np.zeros([4,4])
        self.kalman_R[0,0] = 0.2**2
        self.kalman_R[1,1] = 0.2**2
        self.kalman_R[2,2] = np.power(0.05*np.pi/180,2)
        self.kalman_R[3,3] = np.power(0.09*np.pi/180,2)
        # self.kalman_R[3,3] = 0.00049
        # self.kalman_R[4,4] = 0.00036
        # self.kalman_R[5,5] = np.power(2*np.pi/180,2)

        self.Ts = 0.1
        self.tic_proccess = 0.0
        # print('R_kalman')
        # print(self.kalman_R)
        # print('Q_kalman')
        # print(self.kalman_Q)
        
        #%% Aux variables
        self.radio = 0.064
        self.O3 = np.zeros([3,3])
        self.I3 = np.identity(3)
        self.I6 = np.identity(6)
        self.I8 = np.identity(8)
        self.cont = 0
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.Ts_encL = 0.0
        self.Ts_encL0 = 0.0
        self.encL0 = 0.0
        self.Ts_encR = 0.0
        self.Ts_encR0 = 0.0
        self.encR0 = 0.0
        self.ref_L = 0.0
        self.ref_R = 0.0
        self.tick = 0.0
        
        self.Acc_x2 = 0.0
        self.Acc_x1 = 0.0
        self.Acc_xf_2 = 0.0
        self.Acc_xf_1 = 0.0
        self.Acc_xf = 0.0
        
        self.Acc_y2 = 0.0
        self.Acc_y1 = 0.0
        self.Acc_yf_2 = 0.0
        self.Acc_yf_1 = 0.0
        self.Acc_yf = 0.0
        
        self.Wz2 = 0.0
        self.Wz1 = 0.0
        self.Wzf_2 = 0.0
        self.Wzf_1 = 0.0
        self.Wzf = 0.0
        
        self.times = True
        
        #%% INIT
        print("Starting Kalman node - V3.0.T")
        rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
        rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
        rospy.Subscriber('/WR_ref', Float32, self.Wr_ref_callbaback)
        rospy.Subscriber('/WL_ref', Float32, self.Wl_ref_callbaback)
        # rospy.Subscriber('/enc_L', Int64, self.Wl_callbaback)
        # rospy.Subscriber('/enc_R', Int64, self.Wr_callbaback)
        rospy.Subscriber('/ekf', Odometry, self.ekf_old)
        rospy.Subscriber('/imu/data', Imu, self.calIMU)             #100 Hz
        rospy.Subscriber('/imu/mag', MagneticField, self.calMag)	#100 Hz
        rospy.Subscriber('/gps/odom', Odometry, self.gps_callback)	#500 Hz
        # rospy.Subscriber('/odom_alpha', Odometry, self.odom_callback)	#500 Hz
        
        FILE =  open('ekf_modified.txt','w')
        line = 'x_e' + '\t' +'y_e' + '\t' +'yaw_e' + '\t' +'vx_e' + '\t' +'vy_e' + '\t' +'Bx' + '\t' + 'By'
        line += '\t' +'x_N' + '\t' +'y_E' + '\t' +'yaw' + '\t'
        line += 'y_ICR_r' + '\t' +'y_ICR_l' + '\t' +'x_ICR_v'
        line += '\t' + 'vr' + '\t' + 'vl'
        line += '\t' +'x' + '\t' +'y' + '\t' +'yaw_imu' + '\t'
        line += 'wr' + '\t' + 'wl'
        line += '\t' + 'wr_f' + '\t' + 'wl_f'
        line += '\t' + 'old_x' + '\t' + 'old_y'
        line += '\t' + 'ref_r' + '\t' + 'ref_l'
        FILE.write(line + '\n')
        FILE.close()
        
        Flag_out = False
        self.Flag_start = False
        rate = rospy.Rate(1/self.Ts) # 100hz
        while not Flag_out:
            node_names = rosnode.get_node_names()
            # bag_node = [i for i in node_names if 'xsens' in i]
            bag_node = [i for i in node_names if 'play' in i]
            if (len(bag_node) == 0) and self.Flag_start:
                rospy.signal_shutdown('finish')
                Flag_out = True
            else:
                self.Kalman()
            rate.sleep()
        os.system('python read_ekf_file.py')
        exit()
        
    #%% old EKF
    def ekf_old(self,data):
        self.ekf_x = data.pose.pose.position.x
        self.ekf_y = data.pose.pose.position.y

    #%% Callback GPS
    def gps_callback(self,data):
        if self.flag_gps:
            self.gps_0x = data.pose.pose.position.x
            self.gps_0y = data.pose.pose.position.y
            self.flag_gps = False
        self.gps_x = data.pose.pose.position.x - self.gps_0x
        self.gps_y = data.pose.pose.position.y - self.gps_0y
        # v = data.twist.twist.linear.x
        # angle = data.pose.pose.orientation.z
        # self.d += v
        # self.gps_x = self.d*np.cos(angle*np.pi/180)
        # self.gps_y = self.d*np.sin(angle*np.pi/180)
        
    #%% Callback Encoders
    def Wr_ref_callbaback(self,data):
        self.ref_R = data.data
    
    def Wl_ref_callbaback(self,data):
        self.ref_L = data.data
    
    def Wl_callbaback(self,data):
        #% Displacement old variables
        self.wl_2 = self.wl_1
        self.wl_1 = self.wl
        self.wl_f_2 = self.wl_f_1
        self.wl_f_1 = self.wl_f
        
        # Measurement of sample time of encoders
        self.Ts_encL = rospy.get_time() - self.Ts_encL0
        self.Ts_encL0 = rospy.get_time()
        
        # self.wl = (data.data - self.encL0)/self.Ts_encL
        # self.wl *= self.rpm2rad
        self.wl = data.data
        
        #-----------------------------------------
        # filter Left encoder
        #% Interpolation
        Dw = 0.08*(self.wl - self.wl_1)/self.Ts_encL
        if (abs(Dw) > self.acce):
            Delta_wl_1 = self.wl_1 - self.wl_2
            wl = self.wl_1 + Delta_wl_1
        else:
            wl = self.wl
        #% Low-pass filter
        self.wl_f = 0.06327976*wl
        self.wl_f += 0.12655953*self.wl_1 
        self.wl_f += 0.06327976*self.wl_2
        self.wl_f -= -1.07222953*self.wl_f_1
        self.wl_f -= 0.32534858*self.wl_f_2
        
        #% Displacement old variables
        self.encL0 = data.data
    
    def Wr_callbaback(self,data):
        #% Displacement old variables
        self.wr_2 = self.wr_1
        self.wr_1 = self.wr
        self.wr_f_2 = self.wr_f_1
        self.wr_f_1 = self.wr_f
        
        # Measurement of sample time of encoders
        self.Ts_encR = rospy.get_time() - self.Ts_encR0
        self.Ts_encR0 = rospy.get_time()
        
        # self.wr = (data.data - self.encR0)/self.Ts_encR
        # self.wr *= self.rpm2rad
        self.wr = data.data
        
        #-------------------------------------------
        # filter Rigth encoder
        #% Interpolation
        Dw = 0.08*(self.wr - self.wr_1)/self.Ts_encR
        if (abs(Dw) > self.acce):
            Delta_wr_1 = self.wr_1 - self.wr_2
            wr = self.wr_1 + Delta_wr_1
        else:
            wr = self.wr
        #% Low-pass filter
        self.wr_f = 0.06327976*wr 
        self.wr_f += 0.12655953*self.wr_1 
        self.wr_f += 0.06327976*self.wr_2
        self.wr_f -= -1.07222953*self.wr_f_1
        self.wr_f -= 0.32534858*self.wr_f_2
        
        #% Displacement old variables
        self.encR0 = data.data
        
        
    #%% Callback IMU
    def calIMU(self,data):
        #% Displacement old variables
        self.Acc_x2 = self.Acc_x1
        self.Acc_x1 = self.Acc_x
        self.Acc_xf_2 = self.Acc_xf_1
        self.Acc_xf_1 = self.Acc_xf
        
        self.Acc_y2 = self.Acc_y1
        self.Acc_y1 = self.Acc_y
        self.Acc_yf_2 = self.Acc_yf_1
        self.Acc_yf_1 = self.Acc_yf
        
        self.Wz2 = self.Wz1
        self.Wz1 = self.Wz
        self.Wzf_2 = self.Wzf_1
        self.Wzf_1 = self.Wzf
        
        self.Flag_start = True
        #% Measurement of sample time of IMU
        self.imu_toc= rospy.get_time()
        self.imu_Ts = (self.imu_toc - self.imu_tic)
        self.imu_tic = rospy.get_time()
        #% Data descomposition
        q = data.orientation
        euler = quaternion2euler(q)
        self.imu_pitch = euler[1]
        self.imu_roll  = euler[0]
        #% Quit the initial value of yaw
        if self.flag_imu:
            self.euler1 = euler
            print('Initial imu data')
            print('  Roll:  %0.6f' % self.euler1[0])
            print('  Pitch: %0.6f' % self.euler1[1])
            print('  yaw:   %0.6f' % self.euler1[2])
            self.flag_imu = False
        self.imu_yaw   = euler[2] - self.euler1[2]
        if self.imu_yaw < -np.pi:
            self.imu_yaw += 2.0*np.pi
        elif self.imu_yaw > np.pi:
            self.imu_yaw -= 2.0*np.pi
        self.Acc_x = data.linear_acceleration.x
        self.Acc_y = data.linear_acceleration.y
        self.Wz = data.angular_velocity.z
        
        #% Low-pass filter
        self.Acc_xf = 0.06327976*self.Acc_x
        self.Acc_xf += 0.12655953*self.Acc_x1 
        self.Acc_xf += 0.06327976*self.Acc_x2
        self.Acc_xf -= -1.07222953*self.Acc_xf_1
        self.Acc_xf -= 0.32534858*self.Acc_xf_2
        #% Low-pass filter
        self.Acc_yf = 0.06327976*self.Acc_y
        self.Acc_yf += 0.12655953*self.Acc_y1 
        self.Acc_yf += 0.06327976*self.Acc_y2
        self.Acc_yf -= -1.07222953*self.Acc_yf_1
        self.Acc_yf -= 0.32534858*self.Acc_yf_2
        #% Low-pass filter
        self.Wzf = 0.06327976*self.Wz
        self.Wzf += 0.12655953*self.Wz1 
        self.Wzf += 0.06327976*self.Wz2
        self.Wzf -= -1.07222953*self.Wzf_1
        self.Wzf -= 0.32534858*self.Wzf_2
    
      
    #%% Callback Mag
    def calMag(self,data):
        #% Displacement old variables
        self.mag_yaw_1 = self.mag_yaw
        #% Measurement of sample time of Mag
        self.mag_Ts = (rospy.get_time() - self.mag_tic)
        self.mag_tic = rospy.get_time()
        #% Data descomposition
        self.x_mag = data.magnetic_field.x
        self.y_mag = data.magnetic_field.y
        self.z_mag = data.magnetic_field.z
        self.mag_yaw = math.atan2(self.y_mag,self.x_mag)
        # self.mag_yaw = math.atan(self.y_mag/self.x_mag)
        #% Quit the initial value of yaw
        if (self.flag_mag):
            self.yaw_mag1 = self.mag_yaw
            print('Initial magnetic yaw: %0.6f' % self.yaw_mag1)
            self.flag_mag = False
        self.mag_yaw   = self.mag_yaw - self.yaw_mag1
        if self.mag_yaw < -np.pi:
            self.mag_yaw += 2.0*np.pi
        if self.mag_yaw > np.pi:
            self.mag_yaw -= 2.0*np.pi
  
    #%% Skid-steer model
    # doi = https://doi.org/10.1002/rob.21509
    # sci-hub.se/10.1002/rob.21509
    # The reference frame is rotated, then the ecuations (11) ans (12) changes as:
        # dot_N = Vx*Sin(psi) + Vy*Cos(psi)
        # dot_E = Vx*Cos(phi) - Vy*Sin(psi)
    def SkidSteer(self):
        # Measurement of sample time of encoders
        if self.times:
            self.times = False
        else:
            self.Ts = round(rospy.get_time() - self.tick, 2)
        self.tick = rospy.get_time()
        # print(rospy.get_time() - self.tic_proccess)
        # self.tic_proccess = rospy.get_time()
          
        # print(self.wl,self.wr)
        # [ 0   1   2   3       4       5]
        # [x_N y_E yaw y_ICR_r y_ICR_l x_ICR_v]]
        vl = self.radio*self.wl_f_2
        vr = self.radio*self.wr_f_2
        # print('vr,vl')
        # print(vr,vl)
        
        xN = self.x_model[0,0]
        yE = self.x_model[1,0]
        psi = self.x_model[2,0]
        Yr = self.x_model[3,0]
        Yl = self.x_model[4,0]
        Xv = self.x_model[5,0]
        vr = self.x_model[6,0]
        vl = self.x_model[7,0]
        # print(xN,yE,psi,Yr,Yl,Xv,vr,vl)
        
        den = np.abs(Yl-Yr)
        
        vx = (vr*Yl - vl*Yr)/den
        vy = (vl - vr)*Xv/den
        w = - (vl - vr)/den
        
        s = np.sin(psi)
        c = np.cos(psi)
        # Jacobian computation
        Fk13 = self.Ts*(vx*c - vy*s)
        Fk23 = - self.Ts*(vx*s + vy*c)
        Fk = np.array([[1,0,Fk13],
                       [0,1,Fk23],
                       [0,0,1]])
        # print(Fk)
        
        Fi11 = - vl*s/den + vx*s/den + vy*c/den
        Fi12 = vr*s/den - vx*s/den - vy*c/den
        Fi13 = - w*c
        Fi21 = - vl*c/den + vx*c/den - vy*s/den
        Fi22 = vr*c/den - vx*c/den + vy*s/den
        Fi23 = w*s
        Fi31 = w/den
        Fi32 = - w/den
        Fi = np.array([[Fi11,Fi12,Fi13],
                       [Fi21,Fi22,Fi23],
                       [Fi31,Fi32,0]])
        Fi *= self.Ts
        # print(Fi)
        
        Fv11 = (Yl*s - Xv*c)/den
        Fv12 = (- Yl*s + Xv*c)/den
        Fv21 = (Yl*c + Xv*s)/den
        Fv22 = (- Yl*s - Xv*c)/den
        Fv31 = 1/den
        Fv32 = - 1/den
        Fv = np.array([[Fv11,Fv12],
                       [Fv21,Fv22],
                       [Fv31,Fv32]])
        Fv *= self.Ts
        # print(Fv)
        
        # a -> Fk|Fi|Fv
        a = np.concatenate((Fk,Fi),axis=1) 
        a = np.concatenate((a,Fv),axis=1)
        # print(a)
        # b -> 033|I33|032
        b = np.concatenate((self.O3,self.I3),axis=1)
        b = np.concatenate((b,np.zeros([3,2])),axis=1)
        # print(b)
        # b -> 023|023|I22
        c = np.concatenate((np.zeros([2,6]),np.identity(2)),axis=1)
        # print(c)
        
        F = np.concatenate((a,b))
        # print(F)
        F = np.concatenate((F,c))
        # print('F')
        # print(F)
        # print('P')
        # print(self.P_ICR)
        
        L = self.Ts*self.I8
        
        # self.x_model = F.dot(self.x_model)
        
        aux = F.dot(self.P_ICR) # FP
        # print(aux)
        aux = aux.dot(F.T) # FPF'
        # print(aux)
        aux2 = L.dot(self.Q_model) # LQ
        # print(aux2)
        aux2 = aux2.dot(L.T) # LQL'
        # print(aux2)
        self.P_ICR =  aux + aux2
        # print('P')
        # print(self.kalman_P)
        
        # H = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
        #               [0, 1, 0, 0, 0, 0, 0, 0],
        #               [0, 0, 1, 0, 0, 0, 0, 0],
        #               [0, 0, 0, 0, 0, 0, 1, 0],
        #               [0, 0, 0, 0, 0, 0, 0, 1]])
        H = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1]])
        h = H.dot(self.x_model)
        
        # y = np.array([[self.gps_x],
        #               [self.gps_y],
        #               [self.mag_yaw],
        #               [self.radio*self.wr],
        #               [self.radio*self.wl]])
        y = np.array([[self.gps_x],
                      [self.gps_y],
                      [self.radio*self.wr],
                      [self.radio*self.wl]])
        # print('h')
        # print(h)
        # print('y')
        # print(y)
        
        aux = H.dot(self.P_ICR) # HP
        # print(aux)
        aux = aux.dot(H.T) # HPH'
        # print(aux)
        aux += self.R_model # CPC' + R
        # print(aux)
        aux = np.linalg.inv(aux) # (CPC' + R)^-1
        # print(aux)
        K = self.P_ICR.dot(H.T) # PC'
        # print(K)
        K = K.dot(aux) # PC'(CPC' + R)^-1
        # print('k')
        # print(K)
        
        ye = y - h
        # ye = h - y
        # print('ye')
        # print(ye)
        self.x_model = self.x_model + K.dot(ye)
        
        aux = self.I8 - K.dot(H)
        self.P_ICR = aux.dot(self.P_ICR)
        # print('P')
        # print(self.P_ICR)
        
        if self.x_model[2,0] < -np.pi:
            self.x_model[2,0] += 2.0*np.pi
        elif self.x_model[2,0] > np.pi:
            self.x_model[2,0] -= 2.0*np.pi
        # print('states')
        # print(self.x_model)
        # print('-------------------------------------------------')
        # print('')
        # print('')
        
    #%% Callback Kalman
    def Kalman(self):
        if self.Flag_start:
            while True:
                self.SkidSteer()
                # print(np.linalg.det(self.P_ICR))
                if not np.linalg.det(self.P_ICR) > 0.0001:
                    # print('flag')
                    break
            
            U = np.array([[self.Wzf],
                          [self.Acc_xf],
                          [self.Acc_yf]])
            # print('U')
            # print(U)
            
            self.A_imu[0,3] = self.Ts
            self.A_imu[0,5] = - self.Ts**2
            self.A_imu[1,4] = self.Ts
            self.A_imu[1,6] = - self.Ts**2
            self.A_imu[3,5] = - self.Ts
            self.A_imu[4,6] = - self.Ts
            
            self.B_imu[0,1] = self.Ts**2
            self.B_imu[1,2] = self.Ts**2
            self.B_imu[2,0] = self.Ts
            self.B_imu[3,1] = self.Ts
            self.B_imu[4,2] = self.Ts
            
            # print('A')
            # print(self.A_imu)
            # print('B')
            # print(self.B_imu)
            # print('x_imu')
            # print(self.x_imu)
            
            self.x_imu = self.A_imu.dot(self.x_imu) + self.B_imu.dot(U)
            if self.x_imu[2,0] < -np.pi:
                self.x_imu[2,0] += 2.0*np.pi
            elif self.x_imu[2,0] > np.pi:
                self.x_imu[2,0] -= 2.0*np.pi
            
            # print('x_imu')
            # print(self.x_imu)
            
            yk = self.C_imu.dot(self.x_imu)
            
            h = np.array([[self.x_model[0,0]],
                          [self.x_model[1,0]],
                          [self.x_model[2,0]],
                          [self.mag_yaw]])
            
            # # h = np.array([[self.x_model[0,0]],
            # #               [self.x_model[1,0]],
            # #               [self.imu_yaw],
            # #               [self.x_model[0,0]],
            # #               [self.x_model[1,0]],
            # #               [self.x_model[2,0]]])
            
            # print('y')
            # print(yk)
            # print('h')
            # print(h)
            # print('P')
            # print(self.kalman_P)
            
            aux = self.A_imu.dot(self.kalman_P) # AP
            # print(aux)
            aux = aux.dot(self.A_imu.T) # APA'
            # print(aux)
            self.kalman_P =  aux + self.kalman_Q
            # print('P')
            # print(self.kalman_P)
            
            aux = self.C_imu.dot(self.kalman_P) # CP
            # print(aux)
            aux = aux.dot(self.C_imu.T) # CPC'
            # print(aux)
            aux += self.kalman_R # CPC' + R
            # print(aux)
            aux = np.linalg.inv(aux) # (CPC' + R)^-1
            # print(aux)
            K = self.kalman_P.dot(self.C_imu.T) # PC'
            # print(K)
            K = K.dot(aux) # PC'(CPC' + R)^-1
            # print('k')
            # print(K)
            
            ye = h - yk
            # ye = yk - h
            # print('ye')
            # print(ye)
            self.x_imu = self.x_imu + K.dot(ye)
            if self.x_imu[2,0] < -np.pi:
                self.x_imu[2,0] += 2.0*np.pi
            elif self.x_imu[2,0] > np.pi:
                self.x_imu[2,0] -= 2.0*np.pi
            # print('x_imu~')
            # print(self.x_imu)
            
            aux = np.identity(7) - K.dot(self.C_imu)
            self.kalman_P = aux.dot(self.kalman_P)
            # print('P')
            # print(np.round(self.kalman_P, 4))
            # print('===================================================')
            # print('===================================================')
            # print('')
            line = '\t'.join([str(elem[0]) for elem in self.x_imu])
            line += '\t'
            line += '\t'.join([str(elem[0]) for elem in self.x_model])
            line += '\t' + str(self.gps_x) + '\t' + str(self.gps_y) + '\t' + str(self.mag_yaw)
            line += '\t' + str(self.wr) + '\t' + str(self.wl)
            line += '\t' + str(self.wr_f) + '\t' + str(self.wl_f)
            line += '\t' + str(self.ekf_x) + '\t' + str(self.ekf_y)
            line += '\t' + str(self.ref_R) + '\t' + str(self.ref_L)
            FILE =  open('ekf_modified.txt','a')
            FILE.write(line + '\n')
            FILE.close()
            # self.cont += 1
            # if self.cont == 100:
            #     rospy.signal_shutdown('finish')
            #     exit()
            
        # self.publish_odom(self.x_imu[0,0],self.x_imu[1,0],self.x_imu[2,0],self.x_imu[3,0],self.x_imu[4,0], self.x_imu[5,0])
    
    #%% Callback Publish
    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vy, vth):
        # quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        quat = euler2quaternion(0, 0, cur_theta)
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance
        self.ekf_pub.publish(odom)
#%% Main
if __name__ == '__main__':
    rospy.init_node("ekf2V5_node")
    cv = NodoPos()
