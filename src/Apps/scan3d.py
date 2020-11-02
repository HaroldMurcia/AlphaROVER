#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 10:40:42 2020

@author: sebas
"""

# from numpy import zeros
import rospy
# import rosbag
# import os, sys, getopt
from optparse import OptionParser
# import time as T
# from std_msgs.msg import Float32
# from sensor_msgs.msg import MultiEchoLaserScan, Imu, JointState
# import trajectory_msgs.msg as tm
import numpy as np
# from math import asin, copysign

# %% Default variables
def_sweep_angles = [-13.0*np.pi/18.0, 0.0, np.pi/600.0] # [from to  step] = [130°, 0°, 0.3°]
def_angle_fix = [np.pi/4.0,] # 45°

# %% Functions
def deco_arg(options, args):
    # input angles
    in_angles = [float(i) for i in args]
    
    # output file name
    bagname = 'bags/'+options.filename
    L = len(bagname)
    if not bagname[L-4:L] == '.bag':
        bagname = bagname+'.bag'
        
    # Run scan mode
    if options.scan_mode == True:
        # Check input arguments
        try:
            angle_max, angle_min, max_step = in_angles + def_sweep_angles[len(in_angles):]
        except:
            raise Exception("Too many input arguments for sweep mode, expected arguments must be [from] [to] [step].")
        # sweep mode message
        if options.verbose:
            print("=====================================================")
            print("3D scan mode: sweep")
            print("ROS bag path: %s" % bagname)
            print("Selected sweep angles:")
            print("-    from: %.2f rad" % angle_max)
            print("-    to: %.2f rad" % angle_min)
            print("-    step: %.2f rad" % max_step)
            print("=====================================================")
    elif options.scan_mode == False:
        # Check input arguments
        try:
            angle_fix, = in_angles + def_angle_fix[len(in_angles):]
        except:
            raise Exception("Too many input arguments for fix mode, expected arguments must be [angle].")
        # fix mode message
        if options.verbose:
            print("=====================================================")
            print("3D scan mode: fix")
            print("ROS bag path: %s" % bagname)
            print("Selected fix angle: %.2f" % angle_fix)
            print("=====================================================")
    else:
        raise Exception("Scan mode not selected, use -h to get help.")

def check_topics(topics):
    for topic_name,topic_type in topics:
        print(topic_name)
        print(topic_type)
        
def main():
    topics = rospy.get_published_topics()
    check_topics(topics)
    deco_arg(options, args)
    
# %% Input arguments
parser = OptionParser()

parser.add_option("-s", action="store_true", dest="scan_mode",
                  help="Select the scan mode as 'sweep' for 3D reconstruction.")
parser.add_option("-f", action="store_false", dest="scan_mode",
                  help="Select the scan mode as 'fix' for 3D reconstruction.")

parser.add_option("-o", "--outputfile", dest="filename",
                  help="Name of ROSbag output file", metavar="FILE")

parser.add_option("-q", "--quiet",
                  action="store_false", dest="verbose", default=True,
                  help="Don't print status messages to stdout")

(options, args) = parser.parse_args()


# %% Main program
print(options)
print(args)
print(options.filename)

main()