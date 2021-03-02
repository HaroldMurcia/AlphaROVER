#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 14 17:49:19 2021

__author__ = sebastian.tilaguy@gmail.com
__version__ = "4.0"
__maintainer__ = "Sebastian Tilaguy"
__email__ = "sebastian.tilaguy@gmail.com"
__status__ = "Development"
"""
import rospy
import rosnode
import math
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
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
        self.ekf_pub = rospy.Publisher('/ekf/Odom', Odometry, queue_size=10)
        self.Wl_filt = rospy.Publisher('/ekf/Vl', Float32, queue_size=10)
        self.Wr_filt = rospy.Publisher('/ekf/Vr', Float32, queue_size=10)
        self.Ts = 0.1
        #%% Local model
        self.x_l = np.zeros([7,1]) # [x y yaw vx vy bx by]'
        self.A_l = np.identity(7)
        self.B_l = np.zeros([7,3])
        self.C_l = np.array([[1, 0, 0, 0, 0, 0, 0],
                             [0, 1, 0, 0, 0, 0, 0],
                             [0, 0, 1, 0, 0, 0, 0],
                             [0, 0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 0, 1, 0, 0]])
        # Covariance error matrix
        self.P_l = 100*np.identity(7)
        # Process noise covariance matrix
        self.Q_l = np.zeros([7,7])
        self.Q_l[0,0] = 1**2
        self.Q_l[1,1] = 1**2
        self.Q_l[2,2] = np.power(0.5*np.pi/180,2)
        self.Q_l[3,3] = 0.5**2
        self.Q_l[4,4] = 0.5**2
        self.Q_l[5,5] = 0.01**2
        self.Q_l[6,6] = 0.01**2
        # print('Q_Local')
        # print(self.Q_l)
        # Measurement noise covariance matrix
        self.R_l = np.zeros([5,5])
        self.R_l[0,0] = 0.1**2                        # x_model
        self.R_l[1,1] = 0.1**2                        # y_model
        self.R_l[2,2] = np.power(0.06*np.pi/180,2)    # Yaw_model
        self.R_l[3,3] = 0.08**2                       # vx
        self.R_l[3,3] = 0.08**2                       # vy
        # print('R_Local')
        # print(self.R_l)
        #%% Global model
        self.x_g = np.array([[0],       # x_N
                             [0],       # y_E
                             [0],       # yaw
                             [-2.0],     # y_ICR_r
                             [2.0],    # y_ICR_l
                             [0.2],     # x_ICR_v
                             [0],       # vr
                             [0]])      # vl
        self.C_g = np.array([[0, 0, 1, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 1, 0],
                             [0, 0, 0, 0, 0, 0, 0, 1]])
        self.VN = 0
        self.VE = 0
        self.wz = 0
        self.vx = 0
        self.vy = 0
        self.F = np.identity(8)
        # Covariance error matrix
        self.P_g = 100*np.identity(8)
        # Process noise covariance matrix
        self.Q_g = np.zeros([8,8])
        self.Q_g[0,0] = 0.1**2                  # [m]
        self.Q_g[1,1] = 0.1**2                  # [m]
        self.Q_g[2,2] = np.power(0.8*np.pi/180,2) # [rad]
        self.Q_g[3,3] = 0.01**2                 # [m]
        self.Q_g[4,4] = 0.01**2                 # [m]
        self.Q_g[5,5] = 0.01**2                 # [m]
        self.Q_g[6,6] = 0.07**2                 # [m/s]
        self.Q_g[7,7] = 0.07**2                 # [m/s]
        # print('Q_Global')
        # print(self.Q_g)
        # Measurement noise covariance matrix
        self.R_g = np.zeros([3,3])
        self.R_g[0,0] = np.power(0.06*np.pi/180,2)  # [rad] - IMU
        self.R_g[1,1] = 0.08**2                     # [m/s] - Enc R
        self.R_g[2,2] = 0.08**2                     # [m/s] - Enc L
        # print('R_Global')
        # print(self.R_g)
        #%% Encoder variables
        self.vr_x = 0
        self.vl_x = 0
        #%% IMU variables
        self.flag_imu = True
        self.euler1 = [0,0,0]
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        self.Acc_x = 0
        self.Acc_y = 0
        self.Wz = 0
        self.K_turn = 0
        self.imu_yaw1 = 0.0
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
        
        
        #%% Aux variables
        self.tic_proccess = 0.0
        np.random.seed(1)
        
        self.radio = 0.064    
        self.O3 = np.zeros([3,3])
        self.I3 = np.identity(3)
        self.I6 = np.identity(6)
        self.I8 = np.identity(8)
        self.cont = 0
        
        self.Ts_encR0 = 0.0
        self.encR0 = 0.0
        self.ref_L = 0.0
        self.ref_R = 0.0
        self.tick = 0.0
        
        
        
        self.times = True
        
        #%% INIT
        print("Starting Kalman node - V4.2")
        rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
        rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
        rospy.Subscriber('/WR_ref', Float32, self.Wr_ref_callbaback)
        rospy.Subscriber('/WL_ref', Float32, self.Wl_ref_callbaback)
        rospy.Subscriber('/imu/data', Imu, self.calIMU)             #100 Hz
        rospy.Subscriber('/gps/odom', Odometry, self.gps_callback)	#500 Hz
        
        Flag_out = False
        self.Flag_start = False
        rate = rospy.Rate(1/self.Ts) # 10hz
        while not Flag_out:
            node_names = rosnode.get_node_names()
            node = [i for i in node_names if 'xsens' in i]
            if (len(node) == 0) and self.Flag_start:
                rospy.signal_shutdown('finish')
                Flag_out = True
            else:
                self.Kalman()
            rate.sleep()
        exit()
    
    #%% Callback Kalman
    def Kalman(self):
        if self.Flag_start:
            # Measurement of sample time of kalman
            if self.times:
                self.times = False
            else:
                self.Ts = round(rospy.get_time() - self.tick, 2)
            self.tick = rospy.get_time()
            # print('Kalman sample-time: %.3f' % self.Ts)
            ####################### Local model ##############################
            # while True:
            #     self.SkidSteer()
            #     # print(np.linalg.det(self.P_g))
            #     if not np.abs(np.linalg.det(self.P_g)) > 0.01:
            #         # print('flag')
            #         break
            self.SkidSteer()
            U = np.array([[self.wz],
                          [self.Acc_x],
                          [self.Acc_y]])
            # print('U')
            # print(U)

            self.A_l[0,3] = self.Ts
            self.A_l[0,5] = -(self.Ts**2)
            self.A_l[1,4] = self.Ts
            self.A_l[1,6] = -(self.Ts**2)
            
            self.B_l[0,1] = (self.Ts**2)
            self.B_l[1,2] = (self.Ts**2)
            self.B_l[2,0] = self.Ts
            self.B_l[3,1] = self.Ts
            self.B_l[4,2] = self.Ts
            
            # print('A_local')
            # print(self.A_l)
            # print('B_local')
            # print(self.B_l)
            # print('x_local_(k-1)')
            # print(self.x_l)
            
            self.x_l = self.A_l.dot(self.x_l) + self.B_l.dot(U)
            
            # print('x_local_(k)')
            # print(self.x_l)
            
            # print('P_(k-1)')
            # print(self.P_l)
            aux = self.A_l.dot(self.P_l) # AP
            # print(aux)
            aux = aux.dot(self.A_l.T) # APA'
            # print(aux)
            self.P_l =  aux + self.Q_l
            # print('P_local')
            # print(self.P_l)
            
            aux = self.C_l.dot(self.P_l) # CP
            # print(aux)
            aux = aux.dot(self.C_l.T) # CPC'
            # print(aux)
            aux += self.R_l # CPC' + R
            # print(aux)
            aux = np.linalg.inv(aux) # (CPC' + R)^-1
            # print(aux)
            K = self.P_l.dot(self.C_l.T) # PC'
            # print(K)
            K = K.dot(aux) # PC'(CPC' + R)^-1
            # print('k')
            # print(K)
            
            # Output model
            h = self.C_l.dot(self.x_l)
            # Measured states
            yk = np.array([[self.x_g[0,0]],
                           [self.x_g[1,0]],
                           [self.x_g[2,0]],
                           [self.vx],
                           [self.vy]])
            # print('output model')
            # print(h)
            # print('yk -> [x_gps y_gps vr vl]\'')
            # print(yk)
            
            ye = yk - h
            # print('ye')
            # print(ye)
            self.x_l += K.dot(ye)
            # print('x_locla~')
            # print(self.x_l)
            
            aux = np.identity(7) - K.dot(self.C_l)
            self.P_l = aux.dot(self.P_l)
            # print('P_local')
            # print(np.round(self.P_l, 4))
            # print('===================================================')
            # print('===================================================')
            # print('')
            
        self.publish_odom()
    #%% Skid-steer model
    # doi = https://doi.org/10.1002/rob.21509
    # sci-hub.se/10.1002/rob.21509
    # The reference frame is rotated, then the ecuations (10) to (12) changes:
    def model(self,x):
        xN = x[0,0]
        yE = x[1,0]
        psi = x[2,0]
        yr = x[3,0]
        yl = x[4,0]
        xv = x[5,0]
        vr = x[6,0]
        vl = x[7,0]
        # print(xN,yE,psi,yr,yl,xv,vr,vl)
        
        den = np.abs(yl-yr)
        s = np.sin(psi)
        c = np.cos(psi)
        
        self.vx = (vr*yl - vl*yr)/den
        self.vy = (vl - vr)*xv/den
        self.wz = (vl - vr)/den
        
        self.VN = self.vx*c - self.vy*s
        self.VE = self.vx*s + self.vy*c
        
        # Jacobian computation
        Fk13 = self.Ts*(- self.vy*c - self.vx*s)
        Fk23 = self.Ts*(self.vx*c - self.vy*s)
        Fk = np.array([[1,0,Fk13],
                       [0,1,Fk23],
                       [0,0,1.0]])
        # print(Fk)
        
        Fi11 = self.Ts*(((vr - vl)*(yl*c + xv*s))/(-yr + yl)**2)
        Fi12 = self.Ts*(-((vr - vl)*(yr*c + xv*s))/(-yr + yl)**2)
        Fi13 = self.Ts*(-((vr - vl)*s)/(yr - yl))
        Fi21 = self.Ts*(((vr - vl)*(-xv*c + yl*s))/(-yr + yl)**2)
        Fi22 = self.Ts*(((vr - vl)*(xv*c - yr*s))/(-yr + yl)**2)
        Fi23 = self.Ts*(((vr - vl)*c)/(yr - yl))
        Fi31 = self.Ts*((-vr + vl)/(-yr + yl)**2)
        Fi32 = self.Ts*((vr - vl)/(-yr + yl)**2)
        Fi33 = 0;
        Fi = np.array([[Fi11, Fi12, Fi13],
                       [Fi21, Fi22, Fi23],
                       [Fi31, Fi32, Fi33]])
        # print(Fi)

        Fv11 = self.Ts*((yl*c + xv*s)/(-yr + yl));
        Fv12 = self.Ts*((yr*c + xv*s)/(yr - yl));
        Fv21 = self.Ts*((xv*c - yl*s)/(yr - yl));
        Fv22 = self.Ts*((-xv*c + yr*s)/(yr - yl));
        Fv31 = self.Ts*(1/(yr - yl));
        Fv32 = self.Ts*(1/(-yr + yl));
        Fv = np.array([[Fv11, Fv12],
                       [Fv21, Fv22],
                       [Fv31, Fv32]])
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
        self.F = np.concatenate((F,c))
        # print('F')
        # print(self.F)
    
    def SkidSteer(self):
        
        # Non-linear model
        self.x_g[0,0] += self.Ts*self.VN
        self.x_g[1,0] += self.Ts*self.VE
        self.x_g[2,0] += self.Ts*self.wz
        
        L = self.Ts*self.I8
        
        # Kalman Adjustment
        aux = self.F.dot(self.P_g) # FP
        # print(aux)
        aux = aux.dot(self.F.T) # FPF'
        # print(aux)
        aux2 = L.dot(self.Q_g) # LQ
        # print(aux2)
        aux2 = aux2.dot(L.T) # LQL'
        # print(aux2)
        self.P_g = aux + aux2 # FPF'+LQL'
        # print('P_global')
        # print(self.P_g)
        
        aux = self.C_g.dot(self.P_g) # CP
        # print(aux)
        aux = aux.dot(self.C_g.T) # CPC'
        # print(aux)
        aux += self.R_g # CPC' + R
        # print(aux)
        aux = np.linalg.inv(aux) # (CPC' + R)^-1
        # print(aux)
        K = self.P_g.dot(self.C_g.T) # PC'
        # print(K)
        K = K.dot(aux) # PC'(CPC' + R)^-1
        # print('k')
        # print(K)
        
        h = self.C_g.dot(self.x_g)
        
        y = np.array([[self.imu_yaw],
                      [self.vr_x],
                      [self.vl_x]])
        # print('output model')
        # print(h)
        # print('y -> [x_gps y_gps vr vl]\'')
        # print(y)
        
        ye = y - h
        # print('ye')
        # print(ye)
        
        self.x_g += K.dot(ye)
        # print('x_global~')
        # print(self.x_g)
        
        aux = self.I8 - K.dot(self.C_g)
        self.P_g = aux.dot(self.P_g) # (I - KC)P
        # print('P_global')
        # print(np.round(self.P_g, 4))
        
        self.model(self.x_g)
        # print('-------------------------------------------------')
        # print('')
        # print('')
        
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
        wl = data.data
        self.vl_x = wl*self.radio
    
    def Wr_callbaback(self,data):
        wr = data.data
        self.vr_x = wr*self.radio
            
    #%% Callback IMU
    def calIMU(self,data):
        self.imu_yaw1 = self.imu_yaw
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
            self.Flag_start = True
        self.imu_yaw   = euler[2] - self.euler1[2]
        
        dy = self.imu_yaw - self.imu_yaw1
        if dy > np.pi:
            self.K_turn -= 1
        elif dy < -np.pi:
            self.K_turn += 1
        self.imu_yaw += self.K_turn*2*np.pi
        
        self.Acc_x = data.linear_acceleration.x
        
        self.Acc_y = - data.linear_acceleration.y
        
        self.Wz = data.angular_velocity.z
            
    #%% Callback Publish
    def publish_odom(self):
        # quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        quat = euler2quaternion(0, 0, self.x_l[2,0])
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'kalman_odom'
        odom.pose.pose.position.x = self.x_l[0,0]
        odom.pose.pose.position.y = self.x_l[1,0]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.x_l[3,0]
        odom.twist.twist.linear.y = self.x_l[4,0]
        odom.twist.twist.angular.z = self.wz
        odom.twist.covariance = odom.pose.covariance
        self.ekf_pub.publish(odom)
        self.Wl_filt.publish(self.x_g[6,0])
        self.Wr_filt.publish(self.x_g[7,0])
#%% Main
if __name__ == '__main__':
    rospy.init_node("ekf2V5_node")
    cv = NodoPos()
