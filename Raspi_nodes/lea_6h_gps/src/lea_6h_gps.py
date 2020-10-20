#!/usr/bin/env python3

import serial
# import time
import rospy
import numpy as np # se emplea esta para operar matrices

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class GPS(object):
    def __init__(self):
        #initial param
        self.port = rospy.get_param('/lea_6h_gps/device_port')
        self.baud = rospy.get_param('/lea_6h_gps/baud')
        # Geometria del elipsoide
        self.a = rospy.get_param('/lea_6h_gps/a') # radius of the earth in metters
        self.b = rospy.get_param('/lea_6h_gps/b')
        self.e1 = rospy.get_param('/lea_6h_gps/e1')
        self.e2 = rospy.get_param('/lea_6h_gps/e2')
        # Radio polar de curvatura
        self.c = rospy.get_param('/lea_6h_gps/c')
        # Central meridian of huso
        self.huso = rospy.get_param('/lea_6h_gps/huso') # huso to use in Colombia
        
        self.Latitude = 0.0
        self.Longitude = 0.0
        self.quality = 0
        self.DOP = 0.0
        self.satelites = 0
        self.altitude = 0.0
        self.distance = 0.0
        self.MeasureCounting = 0
        self.time_utc = 0.0
        self.speed_m_s = 0.0
        
        self.x = 0.0
        self.y = 0.0
    
    def GpsTimeSeconds(self, Time_Gps):
        H = float(Time_Gps[0:2])
        M = float(Time_Gps[2:4])
        S = float(Time_Gps[4:9])
        
        Time_seconds = H*3600 + M*60 + S
        return Time_seconds
    
    def GPS_read(self,serial):
        line = str(serial.readline())
        data = line.split(",")
        aux = data[0].split("'")
        data[0] = aux[1]
        if data[0] == "$GPGGA":
            self.time_utc = float(data[1])
            self.Latitude = int(float(data[2])/100.0)
            self.Latitude = self.Latitude + (float(data[2])%100.0)/60.0
            if data[3]=='N':
                self.Latitude = self.Latitude*1.0
            elif data[3]=='S':
                self.Latitude = self.Latitude*-1.0
            self.Longitude = int(float(data[4])/100.0)
            self.Longitude = self.Longitude + (float(data[4])%100.0)/60.0
            if data[5]=='E':
                self.Latitude = self.Longitude*1.0
            elif data[5]=='W':
                self.Longitude = self.Longitude*-1.0
            self.quality = int(data[6])
            self.satelites = float(data[7])
            self.DOP = float(data[8])
            self.altitude = float(data[9])
        if data[0] == "$GPRMC":
            self.Latitude = int(float(data[3])/100.0)
            self.Latitude = self.Latitude + (float(data[3])%100.0)/60.0
            self.Longitude = int(float(data[5])/100.0)
            self.Longitude = self.Longitude + (float(data[5])%100.0)/60.0
            self.speed_m_s = float(data[7]) * 0.514444
            self.time_utc = float(data[1])                
            if self.speed_m_s < 0.5:
                self.speed_m_s = 0.0
                self.MeasureCounting += 1
            if self.MeasureCounting == 1:
                t_0 = self.GpsTimeSeconds(data[1])
            else:
                t_1 = self.GpsTimeSeconds(data[1])
                self.distance = (t_1 - t_0) * self.speed_m_s
                t_0 = t_1
    
    def LLH2GPS(self,Lat,Long):
        Long_rad = Long * (np.pi/180.0)
        Lat_rad = Lat * (np.pi/180.0)
        # Get the central meridian of huso = lamnda0
        lamnda0 = (self.huso * 6.0 - 183.0)*(np.pi/180.0)
        
        #Determination of anglular distance that exist between
        #point Longitude and central meridian of huso
        delta_lambda0 = Long_rad - lamnda0
        
        #Coticchia-Surace ecuations for the direct problem
        #Switch geographics to UTM
        #Estimation of parameters
        
        A = np.cos(Lat_rad) * np.sin(delta_lambda0)
        xi = 0.5 * np.log((1+A)/(1.0-A))
        eta = np.arctan(np.tan(Lat_rad)/np.cos(delta_lambda0)) - Lat_rad
        nu = (self.c*0.9996)/np.sqrt((1.0 + self.e2*np.cos(Lat_rad)*np.cos(Lat_rad)))
        zeta = (self.e2/2.0)*(xi*xi)*(np.cos(Lat_rad)*np.cos(Lat_rad))
        A1 = np.sin(2.0*Lat_rad)
        A2 = A1 * (np.cos(Lat_rad)*np.cos(Lat_rad))
        J2 = Lat_rad + A1/2.0
        J4 = (3*J2 + A2)/4.0
        J6 = (5*J4 + A2 * (np.cos(Lat_rad)*np.cos(Lat_rad)))/3.0
        alpha2 = (3.0/4.0)*(self.e2)
        beta = (5.0/3.0)*(alpha2*alpha2)
        gamma = (35.0/27.0)*(np.power(alpha2,2))
        B_phi = 0.9996 * self.c * (Lat_rad - alpha2 * J2 + beta * J4 - gamma * J6)
        
        self.x = xi*nu*(1+zeta/3.0)+500000.0
        self.y = eta*nu*(1+zeta)+B_phi
        
    def ubicacion(self):
        rospy.init_node("GPS_DATA")
        # pub = rospy.Publisher('/gps/utm', Vector3Stamped, queue_size=10)
        pub1 = rospy.Publisher('/gps/odom', Odometry, queue_size=10)
        pub2 = rospy.Publisher('/gps/data', NavSatFix, queue_size=10)
        gps = serial.Serial(self.port, baudrate = self.baud, timeout=0.1)
        if gps.isOpen():
            rate = rospy.Rate(100) # 100hz
            while not rospy.is_shutdown():  
                try:
                    self.GPS_read(gps)
                    self.LLH2GPS(self.Latitude,self.Longitude)
                except:
                    pass
                # PUBLISH DATA
                gps_msg = NavSatFix()
                gps_msg.header.stamp = rospy.get_rostime()
                gps_msg.header.frame_id = "gps_data"
                gps_msg.status.status = -1
                if self.quality == 1:
                    gps_msg.status.status = 0
                elif self.quality == 4:
                    gps_msg.status.status = 2
                elif self.quality == 5:
                    gps_msg.status.status = 1
                gps_msg.latitude = self.Latitude
                gps_msg.longitude = self.Longitude
                gps_msg.altitude = self.altitude
                
                gps_odm = Odometry()
                gps_odm.header.stamp = rospy.get_rostime()
                gps_odm.header.frame_id = "gps_odom_data"
                gps_odm.child_frame_id = "UTM_coordenates"
                gps_odm.pose.pose.position.x = self.x
                gps_odm.pose.pose.position.y = self.y
                gps_odm.pose.pose.position.z = self.altitude
                
                pub1.publish(gps_odm)
                pub2.publish(gps_msg)
                rate.sleep()
                
if __name__ == '__main__':
    try:
        cv = GPS()
        cv.ubicacion()
    except rospy.ROSInterruptException:
        pass
