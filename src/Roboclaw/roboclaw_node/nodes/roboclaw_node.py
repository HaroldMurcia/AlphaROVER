#!/usr/bin/env python
from math import pi, cos, sin
import os
import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()
	
    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, -cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

class Node:
    def __init__(self):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}
	self.ref_WR = 0
	self.ref_WL = 0
	self.WR = 0 
        self.WL = 0
	self.UiR_1 = 0
        self.errorL_1 = 0
	self.UiL_1 = 0
        self.UiR_1 = 0
	self.Kp = 7.0 #7.0
        self.Ki = 0.5 #2.0
        self.Kd = 3.0 #3.0
	self.errorR_1 = 0
	self.errorR_2 = 0
	self.errorL_2 = 0
	self.errorL_1 = 0
	self.UR_2 = 0
	self.UR_1 = 0
	self.UL_2 = 0
        self.UL_1 = 0
        self.dt = 0.1
	self.value = 0
	self.last_UL = 0
	self.last_UR = 0

        rospy.init_node("roboclaw_node") 
        rospy.on_shutdown(self.shutdown)
        #rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/tty_roboclaw")
        baud_rate = int(rospy.get_param("~baud", "115200"))
	
        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
	
	######################################## TOPICS #############################################
	self.encoder_pub = rospy.Publisher('/encoders', String, queue_size=10)
	self.currents_pub = rospy.Publisher('/currents', String, queue_size=10)
	self.voltage_pub = rospy.Publisher('/voltage', String, queue_size=10)
        self.speed_pub = rospy.Publisher('/speed', String, queue_size=10)
	self.WL_pub = rospy.Publisher('/WL', String, queue_size=10)	#
	self.WR_pub = rospy.Publisher('/WR', String, queue_size=10)
	self.RefL_pub = rospy.Publisher('/RefL', String, queue_size=10)
	self.RefR_pub = rospy.Publisher('/RefR', String, queue_size=10)	#
	self.pid_pub = rospy.Publisher('/pid', String, queue_size=10)
	# TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        roboclaw.SpeedM1M2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "14853.7362"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.38"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()

	################################ SUBSCRIBER ####################################
        #rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/joy", Joy, self.joy_event)
        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def joy_event(self,data):
    	buttons = data.buttons;
    	axes = data.axes
	self.outs(buttons, axes)
	self.move(buttons, axes)
	if(buttons[4]==1):
                G=9.5 # max=9.5
        else:
                G=0.0
	self.ref_WR = G*(axes[4] + axes[0])
	self.ref_WL = G*(axes[4] - axes[0])

    def outs(self,buttons,axes):
        if buttons[4]==1 and buttons[0]==1:
                #Laser ON
                os.system("sudo python /home/ubuntu/Desktop/Mercury/laser_on.py")
        elif buttons[4]==1 and buttons[5]==1:
                #Leds ON
                os.system("sudo python /home/ubuntu/Desktop/Mercury/leds_on.py")
        elif buttons[4]==1 and buttons[1]==1:
                #Lights OFF
                os.system("sudo python /home/ubuntu/Desktop/Mercury/lights_off.py")
#        elif axes[1]==-1 and axes[4]==-1 and axes[2]==-1 and axes[5]==-1:
#                print"Restarting System"
#                os.system("sudo reboot")
#        elif buttons[8]==1 and buttons[1]==1:
#                print"Shutting down system"
#                os.system("sudo poweroff")
	elif axes[6]==-1 and buttons[7]==1:
                #Control Mode
                os.system("sudo python /home/ubuntu/Desktop/Mercury/control_mode.py")
		self.value=1
	elif axes[6]==1 and buttons[7]==1:
                #Manual Mode
                os.system("sudo python /home/ubuntu/Desktop/Mercury/manual_mode.py")
		self.value=0
    def encoders(self):	
        Enc1=roboclaw.ReadEncM1(self.address)
	Enc2=roboclaw.ReadEncM2(self.address)

	enc_L=float(Enc2[1])
        enc_R=float(Enc1[1])
        
	speed1=roboclaw.ReadSpeedM1(self.address)
        speed1=float(speed1[1])
	
	speed2=roboclaw.ReadSpeedM2(self.address)
        speed2=float(speed2[1])

	encoder = "LEFT ENC: "+str(enc_L)+", RIGHT ENC: "+str(enc_R)       
	self.encoder_pub.publish(encoder)	
        
        Wr=(0.0092*speed1)-0.1514  #RPM_R
	Wl=(0.0092*speed2)+0.0126  #RPM_L
	self.WR=((2*pi)/60)*Wr     #rad/s R
	self.WL=((2*pi)/60)*Wl     #rad/s L
	speedM= "Speed_L: "+str(self.WL)+",	Speed_R: "+str(self.WR)+" [rad/s]"
	WL= str(self.WL)
	WR=str(self.WR)
	RefL=str(self.ref_WL)
	RefR=str(self.ref_WR)
	self.speed_pub.publish(speedM)
	
	self.WL_pub.publish(WL)	#
	self.WR_pub.publish(WR)
	self.RefL_pub.publish(RefL)
	self.RefR_pub.publish(RefR)	#
	
	i = roboclaw.ReadCurrents(self.address)
        i_L = roboclaw.ReadCurrents(self.address)[1]
        i_R = roboclaw.ReadCurrents(self.address)[2]

        i_L = 10*float(i_L)
        i_R = 10*float(i_R)

        currents = "Current_L: "+str(i_L)+",	Current_R: "+str(i_R)+" [mA]"
        self.currents_pub.publish(currents)

	volts = float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10.0)
	volts = volts + 0.5

	if (volts<11.2):
		volt = "Check battery level!!: "+str(volts)+" [V]"
		self.voltage_pub.publish(volt)
		print "Check battery level!!!"
	else:
		volt = "Main voltage RBW: "+str(volts)+" [V]"
		self.voltage_pub.publish(volt)

    def WR_Control(self):
    	error = self.ref_WR - self.WR
	Up = self.Kp*error
	Ui = self.Ki*(self.errorR_1)+self.UiR_1
	Ud = self.Kd*(error-self.errorR_1)
	UR = Up+Ui+Ud
	
	if UR > 63:
		UR=63
	elif UR < -63:
		UR=-63
	Ui=UR-Up-Ud
	self.errorR_1 = error
	self.UiR_1 = Ui
	return UR

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

    def move(self, buttons, axes):
    	ML = axes[0];
    	MR = axes[4];
    	if(buttons[4]==1):
        	G=63.0 # max=63
        	
    	else:
        	G=0.0

	U_Trac =int(MR*G)
    	U_Dir = int(ML*G)
    	Motor2 = U_Trac+U_Dir
    	Motor1 = U_Trac-U_Dir
	
    	if(Motor2>63):
        	Motor2=63
    	elif(Motor2<-63):
        	Motor2=-63
    	if(Motor1>63):
        	Motor1=63
    	elif(Motor1<-63):
        	Motor1=-63
    	
	self.last_UR = Motor1
	self.last_UL = Motor2

    def run(self):
        #rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
		if self.value==1:
			
                	try:	
				self.encoders()
                    		UR=self.WR_Control()
		    		UL=self.WL_Control()
				pub_pid = "Left ref: "+str(self.ref_WL)+", Speed_L: "+str(self.WL)+", UL: "+str(UL)+",	Right ref: "+str(self.ref_WR)+", Speed_R: "+str(self.WR)+", UR: "+str(UR)
			        self.pid_pub.publish(pub_pid)
		    		roboclaw.ForwardBackwardM2(self.address, int(63-UR))
		    		roboclaw.ForwardBackwardM1(self.address, int(63+UL)) #Left
                	except OSError as e:
                    		rospy.logerr("Could not stop")
                    		rospy.logdebug(e)
		elif self.value==0:
                        
                        try:
				self.encoders()
                                roboclaw.ForwardBackwardM1(self.address, 63+self.last_UR) #Left
		                roboclaw.ForwardBackwardM2(self.address, 63-self.last_UL)
                                
                        except OSError as e:
                                rospy.logerr("Could not stop")
                                rospy.logdebug(e)
            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                status1, enc1, crc1 = roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                status2, enc2, crc2 = roboclaw.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            #if ('enc1' in vars()) and ('enc2' in vars()):
            #    rospy.logdebug(" Encoders %d %d" % (enc1, enc2))
            #    self.encodm.update_publish(enc1, enc2)

            #    self.updater.update()
            #r_time.sleep()

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = rospy.get_rostime()

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        rospy.logdebug("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks is 0 and vl_ticks is 0:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            else:
                roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)
        except OSError as e:
            rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
            rospy.logdebug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
	node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
