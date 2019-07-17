#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import requests
import time, os
#import RPi.GPIO as GPIO
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)

InternetFail = 0
#urlTest='http://25.21.73.120'
urlTest='http://www.google.com'
#utlTest='http://'+os.environ(ROS_PILOT)
timeoutTest=1
#GPIO.setup(26, GPIO.OUT)
#GPIO.setup(20, GPIO.OUT)
#GPIO.setup(21, GPIO.OUT)
#GPIO.setup(17,GPIO.OUT)
#time.sleep(3)

def talker():
    global InternetFail, urlTest, timeoutTest
    pub = rospy.Publisher('url_Test', String, queue_size=10)
    rospy.init_node('communications', anonymous=True)
    rospy.loginfo("Testing URL")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            r = requests.get(urlTest, timeout=timeoutTest)
            Internet_test = "Internet Ok %s" % rospy.get_time()
            rospy.loginfo(Internet_test)
            pub.publish(Internet_test)
            InternetFail=0
            OK()
        except:
	    pass
            Internet_test = "Internet FAIL"
            rospy.loginfo(Internet_test)
            pub.publish(Internet_test)
            InternetFail=InternetFail+1
            if(InternetFail>=2):
                print("-------------------------------")
                print("No internet 2 times. \n")
                print("-------------------------------")
                noConnection()
        #rate.sleep()

def noConnection():
    print "NO NONNECTION"
    #GPIO.output(21, 1)  # turn on
    #GPIO.output(20, 0)  # turn off
    #time.sleep(0.5)
    #GPIO.output(20, 1)  # turn on
    #GPIO.output(21, 0)  # turn off
    #time.sleep(0.5)
    #GPIO.output(21, 1)  # turn on
    #GPIO.output(20, 0)  # turn on

def OK():
    print "bien"

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
