
#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time
import maestro

servo = maestro.Controller()
servo.setRange(0,0,9000) # Motor Base
servo.setRange(1,0,9000) # Motor Yaw
servo.setRange(2,0,9000) # Joint 3
servo.setRange(3,0,9000) # Joint 2
servo.setRange(4,0,9000) # Gripper
servo.setRange(5,0,9000)
servo.setRange(6,0,9000)
servo.setRange(7,0,9000)
servo.setRange(8,0,9000)
servo.setAccel(0,4)
servo.setAccel(1,4)
servo.setAccel(2,4)
servo.setAccel(3,4)
servo.setAccel(4,4)

P0=3000
P1=9000
P2=4500
P3=3000
P4=5500

arm_state=""

def callback(data):
    buttons = data.buttons;
    axes = data.axes
    move(buttons, axes)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
def move(buttons, axes):
    print "Moving"
    if buttons[7]==1 and axes[7]==1:  #Start + up
	home()
    elif buttons[7]==1 and axes[7]==-1:  #Start + down
	tunnel()
    elif buttons[6]==1:  #Back
	free()
    elif buttons[0]==1 and axes[6]==1: 	#A + RIGHT
	move_arm(0,1,axes[5])
    elif buttons[0]==1 and axes[6]==-1: #A + LEFT
        move_arm(0,-1,axes[5])
    elif buttons[0]==1 and axes[7]==1: 	#A + UP
        move_arm(1,+1,axes[5])
    elif buttons[0]==1 and axes[7]==-1: #A + DOWN
        move_arm(1,-1,axes[5])
    elif buttons[1]==1 and axes[7]==1: 	#B + UP
        move_arm(3,1,axes[5])
    elif buttons[1]==1 and axes[7]==-1: #B + DOWN
        move_arm(3,-1,axes[5])
    elif buttons[1]==1 and axes[6]==1: #B + RIGHT
        move_arm(2,-1,axes[5])
    elif buttons[1]==1 and axes[6]==-1: #B + LEFT
        move_arm(2,1,axes[5])
    elif buttons[3]==1 and axes[6]==1:  #Y + RIGHT
        move_arm(4,1,axes[5])
    elif buttons[3]==1 and axes[6]==-1: #Y + LEFT
        move_arm(4,-1,axes[5])
def home():
    print "home"
    servo.setTarget(0,0)
    servo.setTarget(1,0)
    servo.setTarget(2,0)
    servo.setTarget(3,0)
    servo.setTarget(4,0)
    servo.setTarget(3,3000)
    time.sleep(5) 
    servo.setTarget(0,3000)
    servo.setTarget(1,9000)
    servo.setTarget(2,4500)
    time.sleep(5)
    #servo.setTarget(1,0)
#    goPosition(0,3000)
    servo.setTarget(0,0)
#    goPosition(2,4500)
#    goPosition(4,5500)

def tunnel():
    servo.setTarget(0,3000)
    time.sleep(5)
    servo.setTarget(0,0)
    servo.setTarget(1,6000)
    #servo.setTarget(3,4000)
    #servo.setTarget(2,4500)
    #servo.setTarget(4,5500)
def free():
    print "free"
    servo.setTarget(0,0)
    servo.setTarget(1,0)
    servo.setTarget(2,0)
    servo.setTarget(3,0)
    #servo.setTarget(4,0)
def ARM():
    pub = rospy.Publisher('arm_string', String, queue_size=10)
    rospy.init_node('rover_arm', anonymous=True)
    arm_state =  str([servo.getPosition(0),servo.getPosition(1),servo.getPosition(2),servo.getPosition(3),servo.getPosition(4)])
    #rospy.loginfo(arm_state)
    #pub.publish("arm state"+arm_state)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()
def move_arm(ch,dir,g):
    print "Moviendo"
    #global arm_state
    G=(-g+1.0)/(2.0)*1000.0
    if ch==0:
	print "base"
	servo.setAccel(0,4)
	U = servo.getPosition(0)+dir*G
	if U>9000:
		U=9000
	elif U<1000:
		U=1000
	servo.setTarget(0,int(U))
    elif ch==1:
	servo.setAccel(1,4)
	U = servo.getPosition(1)+dir*G
	if U>9000:
		U=9000
	elif U<1000:
		U=1000
	servo.setTarget(1,int(U))
    elif ch==2:
        servo.setAccel(2,4)
        U = servo.getPosition(2)+dir*G
        if U>9000:
                U=9000
        elif U<1000:
                U=1000
        servo.setTarget(2,int(U))
    elif ch==3:
        servo.setAccel(3,4)
        U = servo.getPosition(3)+dir*G
        if U>9000:
                U=9000
        elif U<1000:
                U=1000
        servo.setTarget(3,int(U))
    elif ch==4:
        servo.setAccel(4,4)
        U = servo.getPosition(4)+dir*G
        if U>9000:
                U=9000
        elif U<1000:
                U=1000
        servo.setTarget(4,int(U))
    arm_state =  str([servo.getPosition(0),servo.getPosition(1),servo.getPosition(2),servo.getPosition(3),servo.getPosition(4)])
    print str(arm_state)
    #rospy.loginfo(arm_state)
    #pub = rospy.Publisher('arm_string', String, queue_size=10)
    #pub.publish(arm_state)
    #print str([servo.getPosition(0),servo.getPosition(1),servo.getPosition(2),servo.getPosition(3),servo.getPosition(4)])
    
if __name__ == '__main__':
    try:
        ARM()
    except rospy.ROSInterruptException:
        pass
