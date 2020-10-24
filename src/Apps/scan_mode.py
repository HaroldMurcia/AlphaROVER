#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 11:59:40 2020

@author: sebas
"""

from numpy import zeros
import rospy
import trajectory_msgs.msg as tm
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class Scan():
    def __init__(self):
        rospy.init_node('Scan_mode')
        rospy.Subscriber('/dynamixel_workbench/joint_trajectory', tm.JointTrajectory, self.motor_angle)
        rospy.Subscriber('/um7', Imu, self.angle_imu)
        rospy.spin()
        
        
    def motor_angle(self,data):
        print(data)
        
    def angle_imu(self,data):
        print(data)
    	

if __name__ == '__main__':
    try:
        Scan()
    except rospy.ROSInterruptException:
        pass
