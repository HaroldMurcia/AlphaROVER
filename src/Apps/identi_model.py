#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 20:24:00 2020

@author: sebas
"""

import rospy
from std_msgs.msg import Float32, Int64
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import KeyValue
from optparse import OptionParser
import numpy as np
import os
import rosnode
import time as T

parser = OptionParser()
parser.add_option("-o", "--outputfile", dest="filename",
                  help="Name of ROSbag output file", metavar="FILE")
parser.add_option("-i", action="store_true", dest="ident", default=False,
                  help="Run the ident routine.")
parser.add_option("-s", action="store_true", dest="small", default=False,
                  help="Run ident-routine for small signal.")
parser.add_option("--step", action="store_true", dest="steps", default=False,
                  help="Run ident-routine for a step signal.")

U_ident1 = np.array([[30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 
                      30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 
                      30, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -25, -25, -25, -25,
                      -25, -25, -25, 25, -25, 25, -25, 25, -25, 25, 25, -25, 
                      -25, 25, 25, -25, -25, -25, 25, -25, -25, -25, 25, -25, 
                      25, 25, -25, 25, -25, -25, 25, 25, 25, -25, -25, 25, -25,
                      -25, -25, -25, 25, -25, -25, 25, -25, 25, -25, -25, 25, 
                      -25, -25, 25, 25, -25, 25, 25, -25, 25, 25, 25, -25, -25,
                      -25, 25, 25, 25, 25, -25, 25, -25, -25, -25, -25, -25, 
                      25, 25, -25, 25, -25, 25, -25, -25, -25, 25, 25, -25, 
                      -25, 25, -25, 25, 25, 25, -25, 25, 25, -25, 25, 
                      21.9278158910840, 13.5368364671754, 1.94581746618692, 
                      -10.0343092394307, -19.5923364331460, -24.5688772127973,
                      -23.9208469141895, -17.9057216762786, -7.96405702560531,
                      3.65207571406030, 14.4051025948500, 22.0276358288169, 
                      24.9873328842899, 22.7699746453866, 15.9355997437173, 
                      5.95316622220148, -5.13631709277292, -15.1483216065896, 
                      -22.1841292267491, -24.9780707524715, -23.1029949305229,
                      -17.0065116085610, -7.88460121183810, 2.57154949734102, 
                      12.5000000000000, 20.2007753898620, 24.4183051416901, 
                      24.5292876295416, 20.6120477364298, 13.3956698744749, 
                      4.10725360746680, -5.74954489813444, -14.6437529142636, 
                      -21.2528337286733, -24.6499009267626, -24.4183051416901,
                      -20.6828986660698, -14.0607443807835, -5.54552005121068,
                      3.65207571406027, 12.2817160931351, 19.2227197438348, 
                      23.6231516433983, 24.9931222977302, 23.2444121472063, 
                      18.6764002859112, 11.9151622257896, 3.81774662245660, 
                      -4.64338035618379, -12.5000000000000, -18.8974794214355,
                      -23.1822107077213, -24.9594447022930, -24.1180756094486,
                      -20.8230310177525, -15.4794046027953, -8.67502819318622,
                      -1.10966470963542, 6.48059071121280, 13.3956698744749,
                      19.0339322156596, 22.9397047063747, 24.8331133423095,
                      24.6214051582989, 22.3927940059853, 18.3953373351800,
                      13.0043783975144, 6.68264123809208, -0.0628317869252142,
                      -6.72299551538168, -12.8250503160961, -17.9640852721959,
                      -21.8264569837335, -24.2041279460146, -25,
                      -24.2249621315342, -21.9878929808683, -18.4802020693867,
                      -13.9566627737589, -8.71430118304543, -3.07098460367923,
                      2.65486632983292, 8.16230334733677, 13.1827937868820,
                      17.4915835128341, 20.9152843084211, 23.3357557945769,
                      24.6905681707963, 24.9704934956481, 24.2145790282158,
                      22.5034095811535, 19.9511731019167, 16.6971129148027,
                      12.8968902676389, 8.71430118304535, 4.31370071864113,
                      -0.146606816860488, -4.51984507944680, -8.67502819318617,
                      -12.5000000000001, -15.9033022071478, -18.8149705088285,
                      -21.1863910429930, -22.9893700772956, -24.2145790282158,
                      -24.8695365700784, -24.9762816088736, -24.5688772127973,
                      -23.6908680375724, -22.3927940059853, -20.7298424086274,
                      -18.7597003370407, -16.5406503271948, -14.1299349137314,
                      -11.5824008779966, -8.94942152156476, -6.27808535416450,
                      -3.61063206453365, -0.984111364089962, 1.56976298823283,
                      4.02459323777699, 6.35914116770603, 8.55706354173926,
                      10.6065660585044, 12.5000000000000, 14.2334229631316,
                      15.8061420836034, 17.2202551486760, 18.4802020693867,
                      19.5923364331460, 20.5645243463139, 21.4057755421137,
                      22.1259097912254, 22.7352600141340, 23.2444121472063,
                      23.6639807411717, 24.0044184478189, 24.2758569513516,
                      24.4879764964248, 24.6499009267626, 24.7701150489425,
                      24.8564011498151, 24.9157916000564, 24.9545346502603,
                      24.9780707524715, 24.9910170047625, 24.9971576077954,
                      24.9994385312735, 24.9999649080815, 25, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, -30, -30, -30, -30, -30, -30, -30, -30,
                      -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30,
                      -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30]])
U_ident2 = np.array([[30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 50, 50, 
                      30, 30, 50, 50, 30, 30, 50, 50, 30, 30, 30, 30, 50, 50, 
                      50, 50, 30, 30, 30, 30, 50, 50, 30, 30, 30, 30, 30, 30, 
                      50, 50, 30, 30, 30, 30, 50, 50, 30, 30, 50, 50, 50, 50, 
                      30, 30, 50, 50, 50, 50, 30, 30, 30, 30, 30, 30, 50, 50, 
                      50, 50, 50, 50, 30, 30, 50, 50, 30, 30, 30, 30, 30, 30, 
                      30, 30, 50, 50, 50, 50, 30, 30, 50, 50, 30, 30, 50, 50, 
                      50, 50]])
U_ident3 = np.concatenate((-20*np.ones([1,10]),
                           -10*np.ones([1,10]),
                           0*np.ones([1,10]),
                           10*np.ones([1,10]),
                           20*np.ones([1,10]),
                           30*np.ones([1,10]),
                           40*np.ones([1,10]),
                           50*np.ones([1,10])),axis=1)

class PID_speed:
    def __init__(self):
        self.Kp = 6.0 #7.0
        self.Ki = 0.5 #2.0
        self.Kd = 3.0 #3.0
        self.mode = 0
        self.errorL_1 = 0.0
        self.UiL_1 = 0.0
        self.errorR_1 = 0.0
        self.UiR_1 = 0.0
        self.ref_WL = 0.0
        self.ref_WR = 0.0
        self.wr = 0.0
        self.wl = 0.
        self.il = 0.0
        self.ir = 0.0
        self.encl = 0.0
        self.encr = 0.0
        self.a = np.arry([0,0,0])
        self.av = np.arry([0,0,0])
        
        topics_list = ['/UL','/UR','/WL','/WL_ref','/WR','/WR_ref','/enc_L','/enc_R',
                       '/iL','/iR','/voltage','/um7','/imu/data']
        self.topic2save = ' '.join(topics_list)
        
        (self.options, self.args) = parser.parse_args()
        
        rospy.init_node("Ident_model")
        
    #     rospy.Subscriber("/ctrl_flag", KeyValue, self.mode_callbaback)
    #     rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
    #     rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
    #     rospy.Subscriber('/WL_ref', Float32, self.Wl_ref)
    #     rospy.Subscriber('/WR_ref', Float32, self.Wr_ref)
    #     rospy.Subscriber('/iL', Float32, self.il_callback)
    #     rospy.Subscriber('/iR', Float32, self.ir_callback)
    #     rospy.Subscriber('/enc_L', Int64, self.encl_callback)
    #     rospy.Subscriber('/enc_R', Int64, self.encr_callback)
    #     rospy.Subscriber('/um7', Imu, self.imu_callback)
    #     rospy.Subscriber('/imu/data', Imu, self.calIMU)
        
    # def calIMU(self,data):
    #     self.av[0] = data.linear_acceleration.x
    #     self.av[1] = data.linear_acceleration.y
    #     self.av[2] = data.linear_acceleration.z
        
    # def imu_callback(self,data):
    #     self.a[0] = data.linear_acceleration.x
    #     self.a[1] = data.linear_acceleration.y
    #     self.a[2] = data.linear_acceleration.z
        
    # def mode_callbaback(self,data):
    #     self.mode = int(data.value)
    
    # def Wl_ref(self,data):
    #     self.ref_WL = data.data
    
    # def Wr_ref(self,data):
    #     self.ref_WR = data.data
    
    # def Wl_callbaback(self,data):
    #     self.wl = data.data
    
    # def Wr_callbaback(self,data):
    #     self.wr = data.data
    
    # def il_callback(self,data):
    #     self.il = data.data
    
    # def ir_callback(self,data):
    #     self.ir = data.data
        
    # def encl_callback(self,data):
    #     self.encl = data.data
    
    # def encr_callback(self,data):
    #     self.encr = data.data
        
    def pub_control(self):
        Ts = 0.1
        r_time = rospy.Rate(1/Ts)
        WL_pub = rospy.Publisher('/UL', Float32, queue_size=10)    #
        WR_pub = rospy.Publisher('/UR', Float32, queue_size=10)
        cont_times = -1
        cont_u = 0
        K = 2
        if self.options.small:
            # U_ident = U_ident2
            U_ident = (U_ident1*0.3)+20
        else:
            U_ident = U_ident1
        if self.options.steps:
            U_ident = U_ident3
            
        print(U_ident)
        L = len(U_ident)
        while not rospy.is_shutdown():
            # print(self.mode)                
            if self.mode == 0:
                UR = self.ref_WR
                UL = self.ref_WL
            else:
                if self.options.ident:
                    if cont_times==-1:
                        # data = pd.DataFrame([],columns=(['u', 'wl', 'wr', 'il', 'ir', 'encL', 'encR']))
                        os.system("rosbag record -O %s %s &" % (self.options.filename, self.topic2save))
                        print('Begining identification test...')
                        cont_times += 1
                        T.sleep(2)
                        
                    u = U_ident[0,int(cont_u)]
                    print(u)
                    UL = float(u)
                    UR = float(u)
                    # S = pd.DataFrame([[u, self.wl, self.wr, self.il, self.ir, self.encl, self.encr]],
                    #                  columns=(['u', 'wl', 'wr', 'il', 'ir', 'encL', 'encR']))
                    # data = data.append(S, ignore_index=True)
                    print('Iteration: ',str(cont_times),
                          ', action: ',str(u),
                          ', Cont: ',str(cont_u),' of ',str(L))
                    if cont_times < K:
                        cont_u += 1
                        if cont_u == L:
                            cont_u = 0
                            cont_times += 1
                    else:
                        print('Finishing identification test...')
                        node_names = rosnode.get_node_names()
                        bag_node = [i for i in node_names if 'record' in i]
                        WL_pub.publish(0.0)   #
                        WR_pub.publish(0.0)
                        os.system("rosnode kill %s" % bag_node[0])
                        # data.to_csv('ident_data.csv',sep='\t')
                        rospy.signal_shutdown('finish')
                        exit()
                        
                else:
                    UR = float(self.args[0])
                    UL = float(self.args[0])
            
            if UR > 63.:
                UR = 63.0
            elif UR < -63.0:
                UR = -63.0
            if UL > 63.0:
                UL = 63.0
            elif UL < -63.0:
                UL = -63.0
            print('Ul: ',str(UL),', Ur: ',str(UR))
            WL_pub.publish(UL)   #
            WR_pub.publish(UR)
            
            r_time.sleep()
            
    
if __name__ == "__main__":
    try:
        node = PID_speed()
        node.pub_control()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")