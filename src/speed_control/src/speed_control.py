#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 20:24:00 2020

@author: sebas
"""

import rospy
from std_msgs.msg import Float32
from diagnostic_msgs.msg import KeyValue
import numpy as np

class PID_speed:
    def __init__(self):
        self.Kp = 8.1410 # 5.109923
        self.Ki = 1.7879 # 3.109923
        self.Kd = 9.2674 # 2.774808
        self.K = 0.8 
        self.mode = 0
        self.errorL_1 = 0.0
        self.UiL_1 = 0.0
        self.errorR_1 = 0.0
        self.UiR_1 = 0.0
        self.ref_vL = 0.0
        self.ref_vR = 0.0
        self.vr = 0.0
        self.vl = 0.0
        # self.Ts_encL = 0.0
        # self.Ts_encR = 0.0
        # self.Ts_encL0 = 0.0
        # self.Ts_encR0 = 0.0
        # self.encL0 = 0.0
        # self.encR0 = 0.0
        # self.rpm2rad = 2.0*np.pi/6533
        self.radio = 0.064
        rospy.init_node("speed_control")
        
        rospy.Subscriber("/ctrl_flag", KeyValue, self.mode_callbaback)
        rospy.Subscriber('/ekf/Vl', Float32, self.Wl_callbaback)
        rospy.Subscriber('/ekf/Vl', Float32, self.Wr_callbaback)
        rospy.Subscriber('/WL_ref', Float32, self.Wl_ref)
        rospy.Subscriber('/WR_ref', Float32, self.Wr_ref)
    
    def mode_callbaback(self,data):
        self.mode = int(data.value)
    
    def Wl_ref(self,data):
        self.ref_vL = data.data * self.radio
    
    def Wr_ref(self,data):
        self.ref_vR = data.data * self.radio
    
    def Wl_callbaback(self,data):
        self.vl = data.data
    
    def Wr_callbaback(self,data):
        self.vr = data.data
    
    def Wl_control(self,ref):
        # print(ref,self.wl)
        error = ref - self.vl
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
        error = ref - self.vr
        
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
        r_time = rospy.Rate(10)
        WL_pub = rospy.Publisher('/UL', Float32, queue_size=10)    #
        WR_pub = rospy.Publisher('/UR', Float32, queue_size=10)
        while not rospy.is_shutdown():
            # print(self.mode)                
            if self.mode == 0:
                self.errorL_1 = 0.0
                self.UiL_1 = 0.0
                self.errorR_1 = 0.0
                self.UiR_1 = 0.0
                UR = self.ref_vR
                UL = self.ref_vL
                if UR > 63:
                    UR = 63
                elif UR < -63:
                    UR = -63
                if UL > 63:
                    UL = 63
                elif UL < -63:
                    UL = -63
            else: # Controlled speed
                UR = self.Wr_control(self.ref_vR)
                UL = self.Wl_control(self.ref_vL)
                
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