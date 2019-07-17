import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time
import maestro

servo = maestro.Controller()
servo.setRange(0,0,9000) # Base
servo.setRange(1,0,9000) # Ext 1
servo.setRange(2,0,9000) # Joint 1 (elbow)
servo.setRange(3,0,9000) # Ext 2
servo.setRange(4,0,9000) # Joint 2 (wrist)
servo.setRange(5,0,9000) # Gripper
servo.setAccel(0,4)
servo.setAccel(1,4)
servo.setAccel(2,4)
servo.setAccel(3,4)
servo.setAccel(4,4)
servo.setAccel(5,4)

def callback(data):
    buttons = data.buttons;
    axes = data.axes
    move(buttons, axes)

def move(buttons, axes):

	if buttons[7]==1 and axes[7]==1:  	#Start + up
		pickup()

	elif buttons[7]==1 and axes[7]==-1:  	#Start + down
		expand()

	elif buttons[6]==1:  			#Back
		free()

	elif buttons[0]==1 and axes[6]==1:	#A + left
		move_arm(0,-1,axes[5])	

	elif buttons[0]==1 and axes[6]==-1:      #A + right
                move_arm(0,1,axes[5])

	elif buttons[0]==1 and axes[7]==1:	 #A + up
		move_arm(1,1,axes[5])

	elif buttons[0]==1 and axes[7]==-1: 	 #A + down
		move_arm(1,-1,axes[5])

	elif buttons[1]==1 and axes[7]==1: 	 #B + up
	        move_arm(2,-1,axes[5])

	elif buttons[1]==1 and axes[7]==-1:	 #B + down
		move_arm(2,1,axes[5])

	elif buttons[3]==1 and axes[7]==1:  	 #Y + up
	        move_arm(3,1,axes[5])

	elif buttons[3]==1 and axes[7]==-1: 	 #Y + down
		move_arm(3,-1,axes[5])	

	elif buttons[1]==1 and axes[6]==1:  	 #B + left
	        move_arm(4,-1,axes[5])

	elif buttons[1]==1 and axes[6]==-1:	 #B + right
		move_arm(4,1,axes[5])

	elif buttons[3]==1 and axes[6]==1:  	 #Y + left
                move_arm(5,1,axes[5])

        elif buttons[3]==1 and axes[6]==-1: 	 #Y + right
                move_arm(5,-1,axes[5])

def move_arm(ch,dir,val):

	G=(-val+1.0)/(2.0)*1000.0

    	if ch==0:
		
		U = servo.getPosition(ch) + (dir*G)		

		if U>8000:
			U=8000
		elif U<2000:
			U=2000
		elif U<4500:
			print "Base left" + "\t" + str(U)
			arm_state = "Base left" + "\t" + str(U)
		elif U>4500:
			print "Base right" + "\t" + str(U)
			arm_state = "Base right" + "\t" + str(U)
			
		servo.setTarget(ch,int(U))

	elif ch==1:
                
                U = servo.getPosition(ch)+dir*G
				
                if U>9000:
                        U=9000
                elif U<6000:
                        U=6000
                elif U<7000:
			print "Extension 1 IN" + "\t" + str(U)
			arm_state = "Extension 1 IN" + "\t" + str(U)
		elif U>7000:
                        print "Extension 1 OUT" + "\t" + str(U)
			arm_state = "Extension 1 OUT" + "\t" + str(U)
			
                servo.setTarget(ch,int(U))

	elif ch==2:
                
                U = servo.getPosition(ch)+dir*G

                if U>5000:
                        U=5000
                elif U<1000:
                        U=1000
                elif U<2600:
                        print "Elbow up" + "\t" + str(U)
			arm_state = "Elbow up" + "\t" + str(U)
                elif U>2600:
                        print "Elbow down" + "\t" + str(U)
			arm_state = "Elbow down" + "\t" + str(U)

                servo.setTarget(ch,int(U))

	elif ch==3:

                U = servo.getPosition(ch)+dir*G
		
                if U>5000:
                        U=5000
                elif U<2000:
                        U=2000
		
                elif U<3500:
                        print "Extension 2 IN" + "\t" + str(U)
			arm_state = "Extension 2 IN" + "\t" + str(U)
                elif U>3500:
                        print "Extension 2 OUT" + "\t" + str(U)
			arm_state = "Extension 2 OUT" + "\t" + str(U)

                servo.setTarget(ch,int(U))

	elif ch==4:
                
                U = servo.getPosition(ch)+dir*G

                if U>9000:
                        U=9000
                elif U<1000:
                        U=1000
                elif dir==-1:
                        print "Wrist left" + "\t" + str(U)
			arm_state = "Wrist left" + "\t" + str(U)
                elif dir==1:
                        print "Wrist right" + "\t" + str(U)
			arm_state = "Wrist right" + "\t" + str(U)

                servo.setTarget(ch,int(U))	

	elif ch==5:

                U = servo.getPosition(ch)+dir*G

                if U>9000:
                        U=9000
                elif U<4300:
                        U=4300
                elif dir==-1:
                        print "Closing gripper" + "\t" + str(U)
			arm_state = "Closing gripper" + "\t" + str(U)
                elif dir==1:
                        print "Opening gripper" + "\t" + str(U)
			arm_state = "Opening gripper" + "\t" + str(U)

                servo.setTarget(ch,int(U))

def expand():
        print "... Deploying ..."
	arm_state = "... Deploying ..."

        servo.setTarget(0,2000)	
	time.sleep(3)
	servo.setTarget(0,0)
	print "Base OK"
	arm_state = "Base OK"

	servo.setTarget(1,9000)
	time.sleep(8)
	servo.setTarget(1,0)
	print "Extension 1 OK"
	arm_state = "Extension 1 OK"

	for joint2 in range(8500,9000,50):
		servo.setTarget(4,joint2)
		time.sleep(0.02)
        servo.setTarget(4,0)
	print "Wrist OK"
	arm_state = "Wrist OK"

	for gripper in range(4300,9000,50):
                servo.setTarget(5,gripper)
                time.sleep(0.02)
        servo.setTarget(5,0)
	print "Gripper OK"
	arm_state = "Gripper OK"

	for joint1 in range(5000,7000,50):
		servo.setTarget(2,joint1)
		time.sleep(0.03)
	servo.setTarget(2,0)
	print "Elbow OK"

	servo.setTarget(3,5000)
        time.sleep(6)
	servo.setTarget(3,0)
        print "Extension 2 OK"
	arm_state = "Extension 2 OK"

	free()
	time.sleep(1)
	print "Extended arm OK"  
	arm_state = "Extended arm OK"  
	
def pickup():
        print "... Picking up ..."
	arm_state = "... Picking up ..."
        
	for gripper in range(8000,4300,-50):
        	servo.setTarget(5,gripper)
                time.sleep(0.02)
	print "Gripper OK"
	arm_state = "Gripper OK"

	servo.setTarget(3,2000)
        time.sleep(6)
	servo.setTarget(3,0)
	print "Extension 2 OK"
	arm_state = "Extension 2 OK"

	for joint2 in range(8000,4500,-50):
                servo.setTarget(4,joint2)
                time.sleep(0.02)
        servo.setTarget(4,0)
	print "Wrist OK"
	arm_state = "Wrist OK"
        
	for joint1 in range(2000,1000,-20):
                servo.setTarget(2,joint1)
		time.sleep(0.1)
        servo.setTarget(2,0)
	servo.setTarget(5,4000)
        print "Elbow OK"
	arm_state = "Elbow OK"

	servo.setTarget(1,4000)
        time.sleep(9)
        servo.setTarget(1,0)
	print "Extension 1 OK"
	arm_state = "Extension 1 OK"

	servo.setTarget(0,7000)
        time.sleep(3)
        servo.setTarget(0,0)	
	print "Base OK"
	arm_state = "Base OK"

	free()
        time.sleep(1)
        print "Picked up OK"
	arm_state = "Picked up OK"

def free():
        print "free"
	arm_state = "free"
        servo.setTarget(0,0)
        servo.setTarget(1,0)
        servo.setTarget(2,0)
        servo.setTarget(3,0)
        servo.setTarget(4,0)
	servo.setTarget(5,0)

def ARM():
    arm_pub = rospy.Publisher('arm_pose', String, queue_size=10)
    rospy.init_node('arm_node', anonymous=True)
    #arm_pub.publish("Arm state:"+arm_state)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ARM()
    except rospy.ROSInterruptException:
        pass

