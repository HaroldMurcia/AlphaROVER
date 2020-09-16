#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import requests
import time, os

InternetFail = 0
#urlTest='http://25.21.73.120'
urlTest='http://www.google.com'
timeoutTest=1
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
    print "NO CONNECTION"
    os.system("sudo python /home/ubuntu/Desktop/Mercury/laser_on.py")
    os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")
    time.sleep(0.2)
    os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")
    time.sleep(0.1)

    os.system("sudo python /home/ubuntu/Desktop/Mercury/laser_on.py")
    os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")
    time.sleep(0.2)
    os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")
    time.sleep(0.1)

    os.system("sudo python /home/ubuntu/Desktop/Mercury/laser_on.py")
    os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")
    time.sleep(0.2)
    os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")

def OK():
    print "OK"

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
