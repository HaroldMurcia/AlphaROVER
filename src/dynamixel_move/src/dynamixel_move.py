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
        self.run()
        
    def run(self):
        pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', tm.JointTrajectory, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            tilt = tm.JointTrajectory()
            tilt.header.frame_id = "joint_trajectory"
            tilt.header.stamp = rospy.Time.now()
            tilt.joint_names = ["tilt"]
            
            jtp = tm.JointTrajectoryPoint()
            jtp.positions = [self.angle]
            jtp.velocities = zeros(len(jtp.positions))
            jtp.time_from_start = rospy.Duration(1)
            
            tilt.points = [jtp]
            
            pub.publish(tilt)
            rate.sleep()
    
    
    def angle_change(self,data):
        self.angle = data.data
        if self.angle<-2.094:
            self.angle = -2.094
        elif self.angle > 0:
            self.angle = 0.0

if __name__ == '__main__':
    try:
        motor()
    except rospy.ROSInterruptException:
        pass