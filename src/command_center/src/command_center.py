#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  6 10:18:16 2020

__author__ = sebastian.tilaguy@gmail.com
__version__ = "1.0"
__maintainer__ = "Sebastian Tilaguy"
__email__ = "sebastian.tilaguy@gmail.com"
__status__ = "Development"
"""

import rospy
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from diagnostic_msgs.msg import KeyValue

class command_center(object):
#%%    Inizialization
    def __init__(self):
        self.EN_ctr = False
        self.ref_WR = 0.0
        self.ref_WL = 0.0
        
        self.ctr_flag = KeyValue()
        self.ctr_flag.key='control model'
        self.ctr_flag.value = '0'
        # --- ctr_flag menu ---
        # 0 -> manual mode
        # 1 -> speed control
        # 2 -> constant lineal speed
        # 3 -> angle control
        self.cam_flag = False
        self.cam_cont = 0
        self.flag_inc = True
        
        rospy.init_node("command_center")
        
        rospy.Subscriber("/joy", Joy, self.joy_event)
        rospy.Subscriber('/cam_shut', Bool, self.cam_shut)
        r_time = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Publishers
            RefL_pub = rospy.Publisher('/WL_ref', Float32, queue_size=10)    #
            RefR_pub = rospy.Publisher('/WR_ref', Float32, queue_size=10)
            flag_pub = rospy.Publisher('/ctrl_flag', KeyValue, queue_size=10)
            
            RefL_pub.publish(self.ref_WL)    #
            RefR_pub.publish(self.ref_WR)
            flag_pub.publish(self.ctr_flag)
            
            r_time.sleep()
    
    def cam_shut(self,data):
        self.cam_flag = data.data
        
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
            if not self.EN_ctr:
                G = 63 # max=9.5
            else:
                G = 31
        else:
            G = 0.0
        
        if self.ctr_flag.value =='2':
            self.ref_WR = G*(0.030913978494623656 + LEFT_ANALOG_HOR)
            self.ref_WL = G*(0.030913978494623656 - LEFT_ANALOG_HOR)
            m = - self.ref_WR/4.0
            self.ref_WR += m*self.cam_cont
            m = - self.ref_WL/4.0
            self.ref_WL += m*self.cam_cont
            if self.cam_flag:
                if self.flag_inc:
                    self.cam_cont +=1
                else:
                    self.cam_cont -=1
                if self.cam_cont == 4:
                    self.flag_inc = False
                elif self.cam_cont == 0:
                    self.flag_inc = True
            print(self.ref_WL,self.ref_WR)
            print(self.cam_flag,self.cam_cont,self.flag_inc)
                
        else:
            self.ref_WR = G*(RIGHT_ANALOG_VER + LEFT_ANALOG_HOR)
            self.ref_WL = G*(RIGHT_ANALOG_VER - LEFT_ANALOG_HOR)
        
        # print(self.ref_WR,self.ref_WL)
        
        if LB==1 and A==1: #Laser ON
            os.system("python ~/catkin_ws/src/AlphaROVER/src/GPIO/laser_on.py'")
            
        elif LB==1 and RB==1: #Leds ON
            os.system("python ~/catkin_ws/src/AlphaROVER/src/GPIO/leds_on.py'")
            
        elif LB==1 and B==1: #Lights OFF
                os.system("python ~/catkin_ws/src/AlphaROVER/src/GPIO/lights_off.py'")
                
        elif LEFT_ANALOG_HOR==-1 and RIGHT_ANALOG_VER==-1 and LT==-1 and RT==-1:
                print("Restarting System")
                rospy.sleep(3)
                os.system("sudo reboot")
                
        elif LOGITECH==1 and B==1:
                print("Shutting down system")
                rospy.sleep(3)
                os.system("sudo poweroff")
                
        elif (LEFT_RIGHT==-1 and START==1 and (not self.EN_ctr)): #Control Mode
            os.system("python ~/catkin_ws/src/AlphaROVER/src/GPIO/control_mode.py'")
            self.EN_ctr = True
            self.ctr_flag.value = '1'
            rospy.loginfo('Speed control - mode: 1')
            
        elif LEFT_RIGHT==1 and START==1 and self.EN_ctr: #Manual Mode
            os.system("python ~/catkin_ws/src/AlphaROVER/src/GPIO/manual_mode.py'")
            self.EN_ctr = False
            self.ctr_flag.value = '0'
            rospy.loginfo('Manual - mode: 0')
            
        elif LEFT_RIGHT==-1 and X==1: #crusing Mode
            os.system("python ~/catkin_ws/src/AlphaROVER/src/GPIO/control_mode.py'")
            self.EN_ctr = True
            self.ctr_flag.value = '2'
            rospy.loginfo('Cruising speed - mode: 2')
            
        elif FRONT_BACK==1 and Y==1:
            print('==========================================')
            print('|               Command menu             |')
            print('------------------------------------------')
            print('| Laser On      | LB + A                 |')
            print('------------------------------------------')
            print('| Lights On     | LB + RB                |')
            print('------------------------------------------')
            print('| Lights Off    | LB + B                 |')
            print('------------------------------------------')
            print('| Restar system | joystick_L_left + LT   |')
            print('|               | + joystick_R_down + RT |')
            print('------------------------------------------')
            print('| Shut down     |  LOGITECH + B          |')
            print('------------------------------------------')
            print('| Control mode  | Arrow_L + START        |')
            print('------------------------------------------')
            print('| Manual mode   | Arrow_R + START        |')
            print('------------------------------------------')
            print('| Cruising mode | Arrow_L + X            |')
            print('------------------------------------------')
    
if __name__ == '__main__':
    cv = command_center()
