#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 10:40:42 2020

@author: sebas
"""

# from numpy import zeros
import rospy
import rosnode
import os
from optparse import OptionParser
import time as T
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import Bool
import trajectory_msgs.msg as tm
import numpy as np
from math import asin, copysign
from datetime import datetime

# %% Default variables
def_sweep_angles = [-13.0*np.pi/18.0, 0.0, np.pi/600.0] # [from to  step] = [130°, 0°, 0.3°]
def_angle_fix = [-np.pi/4.0,] # 45°
min_fix_angle = -np.pi/4.0

flag_all = False
flag_is_saving = False

motor_offset = -np.pi/2.0 #-90°
w_max = np.pi/90.0

flag_print = False
bagname = ""
bag_node = ""
topics_list = ['/gps/odom','/gps/data','/gps/str'
               '/um7',
               '/cam_time',
               '/ekf','/odom_alpha','/ekf2','/velocity',
               '/UL','/UR','/WL','/WL_ref','/WR','/WR_ref','/ctrl_flag',
               '/enc_L','/enc_R',
               '/iL','/iR','/voltage',
               '/diagnostics','/diagnostics_agg','/diagnostics_toplevel_state',
               '/echoes',
               '/imu/data','/imu/mag',
               '/joy', '/SaveData_flag',
               '/dynamixel_workbench/joint_states',
               '/camera/rgb/image_color']
topic2save = ' '.join(topics_list)
# print(topic2save)

pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', tm.JointTrajectory, queue_size=10)
pub_flag = rospy.Publisher('/SaveData_flag', Bool, queue_size=10)
tilt = tm.JointTrajectory()
tilt.header.frame_id = "joint_trajectory"
tilt.joint_names = ["tilt"]
Flag_data = Bool()

# %% Functions
def q2pitch(q):
    sinp = 2* (q.w*q.y - q.z*q.x)
    if np.abs(sinp) >= 1:
        pitch = copysign(np.pi/2,sinp)
    else:
        pitch = asin(sinp)
    return (pitch + motor_offset)
    
def deco_arg(options, args):
    global flag_print, flag_all, bagname
    flag_print = options.verbose
    flag_all = options.save_all
    
    # input angles
    in_angles = [float(i)*np.pi/(-180.0) for i in args]
    
    # output file name
    path = 'bags/'
    if os.path.isdir(path):
        os.system('ls %s -l' % path)
    else:
        os.system('mkdir bags')
        T.sleep(1)
        os.system('ls %s -l' % path)
    # path = '/media/bags/'
    # ## USB_mount?
    # if os.path.ismount(path):
    #     os.system('ls %s -l' % path)
    # else:
    #     os.system('sudo mount -t vfat /dev/sda1 /media/bags/ -o uid=1000,gid=1000')
    #     T.sleep(1)
    #     os.system('ls %s -l' % path)
    
    if options.filename == None:
        now = datetime.now()
        now = now.strftime("%d%m%Y%H%M%S")
        bagname = path + now
    else:
        bagname = path + options.filename
    L = len(bagname)
    if not bagname[L-4:L] == '.bag':
        bagname = bagname+'.bag'
    
    go2top()
    # Run scan mode
    if options.scan_mode == True:
        # Check input arguments
        try:
            angle_max, angle_min, max_step = in_angles + def_sweep_angles[len(in_angles):]
            max_step = np.abs(max_step)
            if (angle_max < def_sweep_angles[0]) or (angle_max > def_sweep_angles[2]):
                angle_max = def_sweep_angles[0]
            if (angle_min < def_sweep_angles[0]) or (angle_min > def_sweep_angles[2]):
                angle_min = def_sweep_angles[1]
            if max_step > def_sweep_angles[2]:
                max_step = def_sweep_angles[2]
        except:
            raise Exception("Too many input arguments for sweep mode, expected arguments must be [from] [to] [step].")
            
        # sweep mode message
        if flag_print:
            print("=====================================================")
            print("3D scan mode: sweep")
            print("ROS bag path: %s" % bagname)
            print("Selected sweep angles:")
            print("-    from: %.6f rad" % angle_max)
            print("-    to: %.6f rad" % angle_min)
            print("-    step: %.6f rad" % max_step)
            print("=====================================================")
        T.sleep(3)
        sweep_scan_mode(angle_max, angle_min, max_step)
            
    elif options.scan_mode == False:
        # Check input arguments
        try:
            angle_fix, = in_angles + def_angle_fix[len(in_angles):]
            if (angle_fix < def_sweep_angles[0]) or (angle_fix > min_fix_angle):
                angle_fix = def_angle_fix[0]
        except:
            raise Exception("Too many input arguments for fix mode, expected arguments must be [angle].")
        # fix mode message
        if flag_print:
            print("=====================================================")
            print("3D scan mode: fix")
            print("ROS bag path: %s" % bagname)
            print("Selected fix angle: %.6f" % angle_fix)
            print("=====================================================")
        T.sleep(3)
        fixed_scan_mode(angle_fix)
        
    else:
        raise Exception("Scan mode not selected, use -h to get help.")

def check_topics():
    topics = rospy.get_published_topics()
    flags = [False,False,False,False]
    for topic_name,topic_type in topics:
        if topic_name == '/um7':
            flags[0] = True
        elif topic_name == '/dynamixel_workbench/joint_states':
            flags[1] = True
        elif topic_name == '/echoes':
            flags[2] = True
        elif topic_name == '/joy':
            flags[3] = True
    if not np.sum(flags[0:4]) == 4:
        raise Exception("The minimum nodes for the scanning process have not been released.")
    return topics
    
def fixed_scan_mode(angle):
    global flag_all
    cur_angle = move_motor(angle)
    if flag_print:
        print("=====================================================")
        print("Scan begining")
        print("Angle to scan: %.6f sec" % cur_angle)
        print("=====================================================")
    # Comand read
    rospy.Subscriber("/joy", Joy, joy_event)
    r_time = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub_flag.publish(Flag_data)
        r_time.sleep()
        

def sweep_scan_mode(angle_max, angle_min, step):
    global flag_all, bag_node
    # print(angle_max, angle_min, step)
    move_motor(angle_max)
    d_scan = np.abs(angle_min - angle_max)
    t_scan = d_scan/w_max
    N = int(d_scan/step)
    #rosbag line
    if flag_all:
        os.system("rosbag record -O %s -a &" % bagname)
    else:
        os.system("rosbag record -O %s %s &" % (bagname, topic2save))
    T.sleep(5)
    node_names = rosnode.get_node_names()
    bag_node = [i for i in node_names if 'record' in i]
    scan_motor(angle_max, angle_min, N, t_scan)
    T.sleep(1)
    os.system("rosnode kill %s" % bag_node[0])

def move_motor(angle):
    global pub, tilt
    
    imu = rospy.wait_for_message('/um7', Imu)
    q = imu.orientation
    pitch_m = q2pitch(q)
    
    motor = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState)
    cur_pos = motor.position

    
    up = 0.0
    ui = 0.0
    
    while np.abs(angle-pitch_m) > 0.02: # error of +/- 2°
        
        e = angle-pitch_m
        up = 0.75*e
        ui = 0.4*e + ui
        u = up + ui + cur_pos[0]
        
        tilt.header.stamp = rospy.Time.now()
        
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = [u]
        jtp.velocities = np.zeros(len(jtp.positions))
        jtp.time_from_start = rospy.Duration(2) # sec - relative to speed
         
        tilt.points = [jtp]
        pub.publish(tilt)
        
        T.sleep(2.1)
        
        imu = rospy.wait_for_message('/um7', Imu)
        q = imu.orientation
        pitch_m = q2pitch(q)
        
        motor = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState)
        cur_pos = motor.position
        
    if flag_print:
        print("IMU_measurement:   %.4f deg" % (pitch_m*180.0/np.pi))
        print("Motor_measurement: %.4f deg" % (cur_pos[0]*180/np.pi))
        print("Error:             %.4f deg" % ((pitch_m - cur_pos[0])*180/np.pi))
        print("---------------------------")
    return pitch_m

def scan_motor(angle1, angle2, N, t):
    global pub, tilt, pub_flag, Flag_data
    
    motor = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState)
    cur_pos = motor.position
    e = angle1-cur_pos[0]
    
    t_step = t/N
    angles = np.linspace(cur_pos, angle2+np.abs(e), int(N))
    
    imu = rospy.wait_for_message('/um7', Imu)
    q = imu.orientation
    pitch_m = q2pitch(q)
    
    if flag_print:
        print("Scanning")
        print("imu: %.4f" % (pitch_m*180.0/np.pi))
        print("motor: %.4f" % (cur_pos[0]*180/np.pi))
        print("---------------------------")
    
    t_i = T.time()
    for i in angles:
        tilt.header.stamp = rospy.Time.now()
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = [i]
        jtp.velocities = [0]
        jtp.time_from_start = rospy.Duration(t_step) # sec - relative to speed
         
        tilt.points = [jtp]
        pub.publish(tilt)
        
        T.sleep(5-t_step)
        Flag_data = True
        pub_flag.publish(Flag_data)
        T.sleep(0.5)
        Flag_data = False
        pub_flag.publish(Flag_data)
        
    imu = rospy.wait_for_message('/um7', Imu)   
    q = imu.orientation
    pitch_m = q2pitch(q)
    
    motor = rospy.wait_for_message('/dynamixel_workbench/joint_states', JointState)
    cur_pos = motor.position
    if flag_print:
        print("imu: %.4f" % (pitch_m*180.0/np.pi))
        print("motor: %.4f" % (cur_pos[0]*180/np.pi))
        print("Scan time: %.2f seg" % (T.time()-t_i))
        print("---------------------------")

def go2top():
    global pub, tilt
    print("=====================================================")
    print("Scan node is beginning")
    print("=====================================================")
    angle = np.pi/(-2.0)
    time_to_move = 3
    for i in [0, 0, angle]:
        
        tilt.header.stamp = rospy.Time.now()
        
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = [i]
        jtp.velocities = np.zeros(1)
        jtp.time_from_start = rospy.Duration(time_to_move) # sec - relative to speed
         
        tilt.points = [jtp]
        pub.publish(tilt)
        
        T.sleep(time_to_move + 0.1)
        
def joy_event(data):
    global flag_all, bag_node, pub_flag, Flag_data, flag_is_saving
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
    
    
    # Rosbag line
    if Y==1 and A==1 and not flag_is_saving: # Rosbag beginning 
        Flag_data = True
        if flag_all:
            os.system("rosbag record -O %s -a &" % bagname)
        else:
            os.system("rosbag record -O %s %s &" % (bagname, topic2save))
        T.sleep(5)
        node_names = rosnode.get_node_names()
        bag_node = [i for i in node_names if 'record' in i]
        print("ROS bag node name: %s" % bag_node)
        flag_is_saving = True
    if Y==1 and B==1 and flag_is_saving : # Rosbag end
        Flag_data = False
        os.system("rosnode kill %s" % bag_node[0])
        rospy.signal_shutdown('finish')
        exit()

def main():
    global topics, Flag_data, pub_flag
    rospy.init_node('scan_node')
    Flag_data = False
    pub_flag.publish(Flag_data)
    topics = check_topics()
    deco_arg(options, args)
    exit()
    
# %% Input arguments
parser = OptionParser()

parser.add_option("-s", action="store_true", dest="scan_mode",
                  help="Select the scan mode as 'sweep' for 3D reconstruction.")
parser.add_option("-f", action="store_false", dest="scan_mode",
                  help="Select the scan mode as 'fix' for 3D reconstruction.")

parser.add_option("-a", action="store_true", dest="save_all", default=False,
                  help="Save all current topics in ROS-master.")

parser.add_option("-o", "--outputfile", dest="filename",
                  help="Name of ROSbag output file", metavar="FILE")

parser.add_option("-q", "--quiet",
                  action="store_false", dest="verbose", default=True,
                  help="Don't print status messages to stdout")

(options, args) = parser.parse_args()


# %% Main program
main()