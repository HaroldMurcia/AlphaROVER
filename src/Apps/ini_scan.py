import rospy
import sys,time,os,datetime
from sensor_msgs.msg import Joy

def callback(data):
    buttons = data.buttons
    axes = data.axes
    move(buttons, axes)

def move(buttons, axes):

	if axes[2]<=-0.9 and buttons[0]==1 and buttons[4]==1:  	#LT + LB + A
		os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")	#Lights ON
                time.sleep(0.2)
                os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")	#Lights OFF
		time.sleep(0.2)
		os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")	#Lights ON
                time.sleep(0.2)
                os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")	#Lights OFF
		time.sleep(0.2)
		os.system("python /home/ubuntu/Desktop/Mercury/scan_data.py 45 -45 0.3")

	elif axes[2]<=-0.9 and buttons[2]==1 and buttons[4]==1:  	#LT + LB + X
		os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")	#Lights ON
                time.sleep(0.2)
                os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")	#Lights OFF
		time.sleep(0.2)
		os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")	#Lights ON
                time.sleep(0.2)
                os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")	#Lights OFF
		time.sleep(0.2)
		os.system("python /home/ubuntu/Desktop/Mercury/scan_mode2.py 45")

def SCAN():
    rospy.init_node('scan_manager')
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        SCAN()
    except rospy.ROSInterruptException:
        pass
