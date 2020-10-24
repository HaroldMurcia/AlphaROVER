#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 11:59:40 2020

@author: sebas
"""

from numpy import zeros
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from sensor_msgs import JointState
import numpy as np
from math import copysign, atan2

class Scan():
    def __init__(self):
        rospy.init_node('Scan_mode')
        rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.motor_angle)
        rospy.Subscriber('/um7', Imu, self.angle_imu)
        rospy.spin()
        
        
    def motor_angle(self,data):
        print(data.position)
        
    def angle_imu(self,data):
        print(data.orientation)
        print(euler_from_quaternion(data.orientation))
    	
    def euler_from_quaternion(self.q):
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        np.a
        roll = atan2(sinr_cosp, cosr_cosp)
    
        # pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if (np.abs(sinp) >= 1):
            pitch = copysign(np.pi/2.0, sinp) # use 90 degrees if out of range
        else:
            pitch = np.asin(sinp)
    
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2.0*(q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

if __name__ == '__main__':
    try:
        Scan()
    except rospy.ROSInterruptException:
        pass
