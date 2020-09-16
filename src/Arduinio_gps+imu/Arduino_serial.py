"""
Created on Sat Aug  8 16:56:14 2020

@author: Sebastian_Tilaguy
"""

import rospy
from sensor_msgs.msg import NavSatFix, Imu
import serial
import tf
from geometry_msgs.msg import Quaternion

ser = serial.Serial(
    port='/dev/tty_Arduino',\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS)

def talker():
    pub_imu = rospy.Publisher('/IMU_LiDAR', Imu, queue_size=10)
    pub_gps = rospy.Publisher('/GPS', NavSatFix, queue_size=10)
    rospy.init_node('IMU_GPS', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    Imu_msg = Imu()
    Gps_msg = NavSatFix()
    print("connected to: " + ser.portstr)
    while not rospy.is_shutdown():
        line = str(ser.readline())
        line = line.replace("\\r\\n'",'')
        data = line.split('$')
#	print(data)
	try:
	        data = data[1].split(',')
	        # print(str(count) + str(': ') + str(data) )
	        if (data[0]=='GPS'):
	            Gps_msg.header.frame_id = 'GPS'
	            Gps_msg.header.stamp = rospy.Time.now()
	            Lat = float(data[1])
	            Long = float(data[2])
	            Ns = float(data[3])
	            Hsnm = float(data[4])
	#            print(str(Lat) + str(', ') + str(Long) + str(', ') + str(Ns) + str(', ') + str(Hsnm))
	            Gps_msg.latitude = Lat
	            Gps_msg.longitude = Long
	            Gps_msg.altitude = Hsnm
	        elif(data[0]=='IMU'):
	            Imu_msg.header.frame_id = 'Imu_Lidar'
	            Imu_msg.header.stamp = rospy.Time.now()
	            Roll = float(data[1])
	            Pitch = float(data[2])
	            Yaw = float(data[3])
	#            print(str(Roll) + str(', ') + str(Pitch) + str(', ') + str(Yaw))
	            q = tf.transformations.quaternion_from_euler(Roll, Pitch, Yaw)
	            Imu_msg.orientation = Quaternion(*q)
	        pub_imu.publish(Imu_msg)
	        pub_gps.publish(Gps_msg)
	except:
		pass
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
      
