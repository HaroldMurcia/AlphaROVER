#!/usr/bin/env python
# https://github.com/arebgun/dynamixel_motor 	libreria dynamixel-ROS
# http://docs.ros.org/kinetic/api/dynamixel_msgs/html/msg/JointState.html 	comandos

import rospy
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
import time

#class NodoDyna(object): # Crea clase

#	def __init__(self):
#	    self.pub = rospy.Publisher('/motor_controller/command', Float64, queue_size=10)
#	    self.pos_inicial = 3.05
#	    self.pos_final = 2.28
#	    self.pos_centro = 2.62
#	    self.mover_final()
#	    print "1"
#	    rospy.spin()

def mover_inicio():
    pub.publish(pos_inicial)
    print pos_inicial

def mover_final():
    pub.publish(2.28)
    print pos_final

def mover_centro():
    pub.publish(pos_centro)    
    print pos_centro
	
def dyn():
    global pub
    global pos_inicial
    global pos_final
    global pos_centro
    pub = rospy.Publisher('/motor_controller/command', Float64, queue_size=10)
    rospy.init_node("move_dynamixel")
    pos_inicial = 3.05
    pos_final = 2.28
    pos_centro = 2.62
    mover_inicio()
    time.sleep(2)
    mover_centro()
    time.sleep(2)
    mover_final()
    time.sleep(2)

    pos = Float64()
	
	    #arm_pub = rospy.Publisher('arm_string', String, queue_size=10)
	    #rospy.init_node('rover_arm', anonymous=True)
	    #arm_state =  str([servo.getPosition(0),servo.getPosition(1)])
	    #pub.publish("arm state"+arm_state)
	    #rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()
	
if __name__ == '__main__':
    try:
       dyn()
    except rospy.ROSInterruptException:
       pass

#if __name__ == '__main__':
#	try:
#		rospy.init_node("move_dynamixel")
#		NodoDyna()
#	except rospy.ROSInterruptException:
#	        pass
#	print "Exiting Dynamixel"

