#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# license removed for brevity
from numpy import  zeros
import rospy
import trajectory_msgs.msg as tm
from std_msgs.msg import Float32


class motor():
    def __init__(self):
        rospy.init_node('Dinamixel_move_control')
        rospy.Subscriber('/dynamixel_angle', Float32, self.angle_change)
        self. angle =0.0
        self.pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', tm.JointTrajectory, queue_size=10)
        self.run()
        
    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
    
    
    def angle_change(self,data):
        self.angle = data.data
        if self.angle<-2.094:
            self.angle = -2.094
        elif self.angle > 0:
            self.angle = 0.0
        tilt = tm.JointTrajectory()
        tilt.header.frame_id = "joint_trajectory"
        tilt.header.stamp = rospy.Time.now()
        tilt.joint_names = ["tilt"]
        
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = [self.angle]
        jtp.velocities = [0.5]
        jtp.effort = [0.3]
        jtp.time_from_start = rospy.Duration(2)
        
        tilt.points = [jtp]
        
        self.pub.publish(tilt)

if __name__ == '__main__':
    try:
        motor()
    except rospy.ROSInterruptException:
        pass
