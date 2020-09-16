# -*- coding: utf-8 -*-
"""
Created on Mon Sep 14 20:24:00 2020

@author: sebas
"""

from math import pi
#from math import cos, sin
import os
import rospy
#import tf
#from geometry_msgs.msg import Quaternion, Twist
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class PID_speed:
    def __init__(self):
        self.Kp = 7.0 #7.0
        self.Ki = 0.5 #2.0
        self.Kd = 3.0 #3.0
        rospy.init_node("Speed Control")
        
    def WL_Control(self):
        error = self.ref_WL - self.WL
        Up = self.Kp*error
        Ui = self.Ki*self.errorL_1+self.UiL_1
        Ud = self.Kd*(error-self.errorL_1)
        UL = Up+Ui+Ud
        
        if UL > 63:
            UL=63
        elif UL < -63:
            UL=-63
        
        Ui=UL-Up-Ud
        self.errorL_1 = error
        self.UiL_1 = Ui
        return UL

    def joy_event(self,data):
        # Charectization GamePad Logitech F710
        # READ BUTTONS
        A = data.buttons[0]
        B = data.buttons[1]
        X = data.buttons[2]
        Y = data.buttons[3]
        LB = data.buttons[4]
        RB = data.buttons[5]
        BACK = data.buttons[6]
        START = data.buttons[7]
        LOGITECH = data.buttons[8]
        ANALOG_L = data.buttons[9]
        ANALOG_R = data.buttons[10]
        # READ Axes
        LEFT_ANALOG_HOR = data.axes[0] # <<(+)
        LEFT_ANALOG_VER = data.axes[1] # ^^(+)
        LT = data.axes[2] #[1 -1]
        RIGHT_ANALOG_HOR = data.axes[3] # <<(+)
        RIGHT_ANALOG_VER = data.axes[4] # ^^(+)
        RT = data.axes[5] #[1 -1]
        LEFT_RIGHT = data.axes[6] # left=1, right=-1
        FRONT_BACK = data.axes[7] # front=1, back=-1
        if(LB==1):
            G=9.5 # max=9.5
        else:
            G=0.0
        self.ref_WR = G*(RIGHT_ANALOG_VER + LEFT_ANALOG_HOR)
        self.ref_WL = G*(RIGHT_ANALOG_VER - LEFT_ANALOG_HOR)
        print(self.ref_WR,self.ref_WL)
        if LEFT_RIGHT==-1 and START==1 and not self.EN_ctr: #Control Mode
            os.system("sudo python /home/ubuntu/Desktop/Mercury/control_mode.py")
            self.EN_ctr = True
        elif LEFT_RIGHT==1 and START==1 and self.EN_ctr: #Manual Mode
            os.system("sudo python /home/ubuntu/Desktop/Mercury/manual_mode.py")
            self.EN_ctr = False
        self.move(LB,RIGHT_ANALOG_VER,LEFT_ANALOG_HOR)
    
if __name__ == "__main__":
    try:
        PID_speed.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")