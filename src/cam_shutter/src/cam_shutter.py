#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  6 17:14:14 2020

__author__ = sebastian.tilaguy@gmail.com
__version__ = "1.0"
__maintainer__ = "Sebastian Tilaguy"
__email__ = "sebastian.tilaguy@gmail.com"
__status__ = "Development"
"""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MultiEchoLaserScan, TimeReference
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Float32, Bool
from math import atan, sqrt
import time as T
import os

class NodoPos(object):
    def __init__(self):
        # Movement spesifications
        self.overlap = rospy.get_param('/cam_shutter/overlap')
        rospy.logwarn('The Overlap parm selected was %.2f' % self.overlap)
        
        # Camera spesifications
        self.cam_ref = rospy.get_param('/cam_shutter/Cam_ref')
        self.Focal_len = float(rospy.get_param('/cam_shutter/Focal_len'))
        self.Sensor_dim_h = float(rospy.get_param('/cam_shutter/Sensor_dim_h'))
        self.Sensor_dim_w = float(rospy.get_param('/cam_shutter/Sensor_dim_w'))
        self.Pixel_size = float(rospy.get_param('/cam_shutter/Pixel_size'))
        self.Depth_filed = float(rospy.get_param('/cam_shutter/Depth_filed'))
        
        rospy.Subscriber('/ekf', Odometry, self.EKF_calback)
        rospy.Subscriber('/echoes', MultiEchoLaserScan, self.Lidar_calback)
        rospy.Subscriber("/ctrl_flag", KeyValue, self.mode_callbaback)
        rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
        rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
        
        self.photo_cont = 0
        self.t_photo = 0.0
        self.t_source = ""
        self.dk = 0.0
        self.r = 3.0
        self.mode = 0
        self.flag_k = True
        
        self.x1 = 0.0
        self.y1 = 0.0
        
        self.wl = 0.0
        self.wr = 0.0
        
        
        rate = rospy.Rate(100) # 100hz
        pub = rospy.Publisher('/cam_time', TimeReference, queue_size=10)
        pub_flag = rospy.Publisher('/cam_shut', Bool, queue_size=10)
        d = self.Cam_calculation(self.r)
        d_ = d*(1-self.overlap)
        flag = False
        while not rospy.is_shutdown():
            print(flag)
            if (self.dk >= d_) and (self.mode == 2):
                flag = True
                if (self.wl==0.0) and (self.wr==0.0):
                    print('photo')
                    os.system('echo 1 > /sys/class/gpio/gpio38/value')
                    # take picture
                    t_photo = TimeReference()
                    t_photo.header.seq = self.photo_cont
                    t_photo.header.stamp = rospy.Time.now()
                    t_photo.header.frame_id = self.cam_ref
                    t_photo.time_ref = self.t_photo
                    t_photo.source = self.t_source
                    
                    pub.publish(t_photo)
                    
                    self.photo_cont = self.photo_cont + 1
                    self.flag_k = True
                    self.dk = 0
                    d = self.Cam_calculation(self.r)
                    d_ = d*(1-self.overlap)
                    flag = False
                    os.system('echo 0 > /sys/class/gpio/gpio38/value')
                    print(self.dk, d_)
            pub_flag.publish(flag)
            rate.sleep()
                
    def EKF_calback(self,data):
        self.t_photo = data.header.stamp
        # print(self.t_photo)
        self.t_source = data.header.frame_id
        # print(self.t_source)
        
        dx = data.pose.pose.position.x
        dy = data.pose.pose.position.y
        if (self.flag_k):
            self.x1 = dx
            self.y1 = dy
            self.flag_k = False
        
        x = dx - self.x1
        y = dy - self.y1
        # print(x,y)
        
        self.dk = sqrt(x*x + y*y)
        # print(self.dk)
        
    def Wl_callbaback(self,data):
        data = data.data*10
        self.wl = (int(data))/10.0
    
    def Wr_callbaback(self,data):
        data = data.data*10
        self.wr = (int(data))/10.0
    
    def Lidar_calback(self,data):
        ranges = data.ranges
        N = len(ranges)
        # print(N)
        # print(N, int(N/2.0), str(ranges[int(N/2.0)]))
        aux = str(ranges[int(N/4.0)]).split("[")
        # print(aux)
        aux = str(aux[1]).strip("]")
        # print(aux)
        aux = aux.split(",")
        # print(aux)
        self.r = float(aux[0]) + 0.43
        # print(self.r)
        
        if self.r <= self.Depth_filed:
            rospy.logwarn('The object is too close to the sensor')
        
    
    def Cam_calculation(self,r):
        imag_w = self.Sensor_dim_w*r/self.Focal_len
        imag_h = self.Sensor_dim_h*r/self.Focal_len
        # print(imag_w,imag_h)
        
        angle_w = 2*atan(self.Sensor_dim_w/(2.0*self.Focal_len))
        angle_h = 2*atan(self.Sensor_dim_h/(2.0*self.Focal_len))
        # print(angle_w,angle_h)
        
        return imag_h
    
    def mode_callbaback(self,data):
        self.mode = int(data.value)

if __name__ == '__main__':
    rospy.init_node("cam_shuetter")
    cv = NodoPos()