#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 20:24:00 2020

@author: sebas
"""

import rospy
from std_msgs.msg import Float32, Int64
from diagnostic_msgs.msg import KeyValue
from optparse import OptionParser
import numpy as np
import os
import rosnode
import time as T
import pandas as pd

parser = OptionParser()
parser.add_option("-o", "--outputfile", dest="filename",
                  help="Name of ROSbag output file", metavar="FILE")
parser.add_option("-i", action="store_true", dest="ident", default=False,
                  help="Run the ident routine.")
parser.add_option("-s", action="store_true", dest="small", default=False,
                  help="Run ident-routine for small signal.")
parser.add_option("--step", action="store_true", dest="steps", default=False,
                  help="Run ident-routine for a step signal.")

U_ident1 = np.array([[-60, -60, -60, -60, -60, -60, -60, 60, -60, 60, -60, 60,
                    -60, 60, 60, -60, -60, 60, 60, -60, -60, -60, 60, -60, -60,
                    -60, 60, -60, 60, 60, -60, 60, -60, -60, 60, 60, 60, -60,
                    -60, 60, -60, -60, -60, -60, 60, -60, -60, 60, -60, 60,
                    -60, -60, 60, -60, -60, 60, 60, -60, 60, 60, -60, 60, 60,
                    60, -60, -60, -60, 60, 60, 60, 60, -60, 60, -60, -60, -60,
                    -60, -60, 60, 60]])
U_ident2 = np.array([[-10, -10, -10, -10, -10, -10, -10, 10, -10, 10, -10, 10,
                    -10, 10, 10, -10, -10, 10, 10, -10, -10, -10, 10, -10, -10,
                    -10, 10, -10, 10, 10, -10, 10, -10, -10, 10, 10, 10, -10,
                    -10, 10, -10, -10, -10, -10, 10, -10, -10, 10, -10, 10,
                    -10, -10, 10, -10, -10, 10, 10, -10, 10, 10, -10, 10, 10,
                    10, -10, -10, -10, 10, 10, 10, 10, -10, 10, -10, -10, -10,
                    -10, -10, 10, 10]])
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
        
        topics_list = ['/UL','/UR','/WL','/WL_ref','/WR','/WR_ref','/ctrl_flag','/enc_L','/enc_R',
                            '/iL','/iR','/voltage']
        self.topic2save = ' '.join(topics_list)
        
        (self.options, self.args) = parser.parse_args()
        
        rospy.init_node("Ident_model")
        
        rospy.Subscriber("/ctrl_flag", KeyValue, self.mode_callbaback)
        rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
        rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
        rospy.Subscriber('/WL_ref', Float32, self.Wl_ref)
        rospy.Subscriber('/WR_ref', Float32, self.Wr_ref)
        rospy.Subscriber('/iL', Float32, self.il_callback)
        rospy.Subscriber('/iR', Float32, self.ir_callback)
        rospy.Subscriber('/enc_L', Int64, self.encl_callback)
        rospy.Subscriber('/enc_R', Int64, self.encr_callback)
        
    def mode_callbaback(self,data):
        self.mode = int(data.value)
    
    def Wl_ref(self,data):
        self.ref_WL = data.data
    
    def Wr_ref(self,data):
        self.ref_WR = data.data
    
    def Wl_callbaback(self,data):
        self.wl = data.data
    
    def Wr_callbaback(self,data):
        self.wr = data.data
    
    def il_callback(self,data):
        self.il = data.data
    
    def ir_callback(self,data):
        self.ir = data.data
        
    def encl_callback(self,data):
        self.encl = data.data
    
    def encr_callback(self,data):
        self.encr = data.data
        
    def pub_control(self):
        Ts = 0.1
        r_time = rospy.Rate(1/Ts)
        WL_pub = rospy.Publisher('/UL', Float32, queue_size=10)    #
        WR_pub = rospy.Publisher('/UR', Float32, queue_size=10)
        cont_times = -1
        cont_u = 0
        K = 2
        if self.options.small:
            U_ident = U_ident2
        else:
            U_ident = U_ident1
        if self.options.steps:
            U_ident = U_ident3
            k = 1
        print(U_ident)
        while not rospy.is_shutdown():
            # print(self.mode)                
            if self.mode == 0:
                UR = self.ref_WR
                UL = self.ref_WL
            else: # constant speed
                if self.options.ident:
                    if cont_times==-1:
                        data = pd.DataFrame([],columns=(['u', 'wl', 'wr', 'il', 'ir', 'encL', 'encR']))
                        os.system("rosbag record -O %s %s &" % (self.options.filename, self.topic2save))
                        print('Begining identification test...')
                        cont_times += 1
                        T.sleep(2)
                        
                    u = U_ident[0,int(cont_u)]
                    print(u)
                    UL = float(u)
                    UR = float(u)
                    S = pd.DataFrame([[u, self.wl, self.wr, self.il, self.ir, self.encl, self.encr]],
                                     columns=(['u', 'wl', 'wr', 'il', 'ir', 'encL', 'encR']))
                    data = data.append(S, ignore_index=True)
                    print(cont_times,cont_u,int(cont_u))
                    if cont_times < K:
                        cont_u += 0.5
                        if cont_u == 79.5:
                            cont_u = 0.0
                            cont_times += 1
                    else:
                        print('Finishing identification test...')
                        node_names = rosnode.get_node_names()
                        bag_node = [i for i in node_names if 'record' in i]
                        WL_pub.publish(0.0)   #
                        WR_pub.publish(0.0)
                        os.system("rosnode kill %s" % bag_node[0])
                        data.to_csv('ident_data.csv',sep='\t')
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
            print(UL,UR)
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