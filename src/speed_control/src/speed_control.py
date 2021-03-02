#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 20:24:00 2020

@author: sebas
"""

import rospy
from std_msgs.msg import Float32, Bool, Int64
from diagnostic_msgs.msg import KeyValue
import numpy as np

class PID_speed:
    def __init__(self):
        self.Kp = 8.1410 # 8.1410 #7.0
        self.Ki = 1.7879 # 1.7879 #0.5
        self.Kd = 9.2674 # 9.2674 #3.0
        self.K = 0.8 
        self.mode = 0
        self.errorL_1 = 0.0
        self.UiL_1 = 0.0
        self.errorR_1 = 0.0
        self.UiR_1 = 0.0
        self.ref_WL = 0.0
        self.ref_WR = 0.0
        self.wr = 0.0
        self.wl = 0.0
        self.cam_flag = False
        self.Ts_encL = 0.0
        self.Ts_encR = 0.0
        self.Ts_encL0 = 0.0
        self.Ts_encR0 = 0.0
        self.encL0 = 0.0
        self.encR0 = 0.0
        self.rpm2rad = 2.0*np.pi/6533
        
        rospy.init_node("speed_control")
        
        rospy.Subscriber("/ctrl_flag", KeyValue, self.mode_callbaback)
        # rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
        # rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
        rospy.Subscriber('/enc_L', Int64, self.Wl_callbaback)
        rospy.Subscriber('/enc_R', Int64, self.Wr_callbaback)
        rospy.Subscriber('/WL_ref', Float32, self.Wl_ref)
        rospy.Subscriber('/WR_ref', Float32, self.Wr_ref)
        rospy.Subscriber('/cam_shut', Bool, self.cam_shut)
        
    def cam_shut(self,data):
        self.cam_flag = data.data
    
    def mode_callbaback(self,data):
        self.mode = int(data.value)
    
    def Wl_ref(self,data):
        self.ref_WL = data.data
    
    def Wr_ref(self,data):
        self.ref_WR = data.data
    
    def Wl_callbaback(self,data):
        # Measurement of sample time of encoders
        self.Ts_encL = rospy.get_time() - self.Ts_encL0
        self.Ts_encL0 = rospy.get_time()
        
        self.wl = (data.data - self.encL0)/self.Ts_encL
        self.wl *= self.rpm2rad
        self.encL0 = data.data
    
    def Wr_callbaback(self,data):
        # Measurement of sample time of encoders
        self.Ts_encR = rospy.get_time() - self.Ts_encR0
        self.Ts_encR0 = rospy.get_time()
        
        self.wr = (data.data - self.encR0)/self.Ts_encR
        self.wr *= self.rpm2rad
        self.encR0 = data.data
    
    def Wl_control(self,ref):
        # print(ref,self.wl)
        error = ref - self.wl
        # print('e(k) = %f' % error)
        
        Up = self.Kp*error
        Ui = self.Ki*self.errorL_1 + self.UiL_1
        Ud = self.Kd*(error - self.errorL_1)
        U = self.K*(Up + Ui + Ud)
        # print('refL(k) = %f' % ref)
        # print('ul(k) = %f' % U)
        
        if U > 63:
            U = 63
        elif U < -63:
            U = -63
        
        Ui = U - Up - Ud
        self.errorL_1 = error
        self.UiL_1 = Ui
        
        return U
        
    def Wr_control(self,ref):
        error = ref - self.wr
        
        Up = self.Kp*error
        Ui = self.Ki*self.errorR_1 + self.UiR_1
        Ud = self.Kd*(error - self.errorR_1)
        U = self.K*(Up + Ui + Ud)
        # print('refR(k) = %f' % ref)
        # print('uR(k) = %f' % U)
        
        if U > 63:
            U = 63
        elif U < -63:
            U = -63
        
        Ui = U - Up - Ud
        self.errorR_1 = error
        self.UiR_1 = Ui
        
        return U
        
    def pub_control(self):
        r_time = rospy.Rate(100)
        WL_pub = rospy.Publisher('/UL', Float32, queue_size=10)    #
        WR_pub = rospy.Publisher('/UR', Float32, queue_size=10)
        while not rospy.is_shutdown():
            
            # print(self.mode)                
            if self.mode == 0:
                self.errorL_1 = 0.0
                self.UiL_1 = 0.0
                self.errorR_1 = 0.0
                self.UiR_1 = 0.0
                UR = self.ref_WR
                UL = self.ref_WL
                if UR > 63:
                    UR = 63
                elif UR < -63:
                    UR = -63
                if UL > 63:
                    UL = 63
                elif UL < -63:
                    UL = -63
            else: # constant speed
                UR = self.Wr_control(self.ref_WR)
                UL = self.Wl_control(self.ref_WL)
                
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