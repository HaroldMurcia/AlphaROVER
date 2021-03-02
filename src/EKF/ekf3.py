import rospy
import rosnode
import math
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, MagneticField
# import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from datetime import date

def quaternion2euler(q):
    ### q = {q.x, q.y, q.z, q.w}
    # roll
    sq = 2.0 * (q.z * q.x + q.y * q.w)
    cq = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll =  math.atan2(sq, cq)
    # pitch
    sq = 2.0 * (q.w * q.y - q.z * q.x)
    if np.abs(sq) >=1:
        pitch = math.copysign(np.pi/2.0, sq) # use 90 degrees if out of range
    else:
        pitch = math.asin(sq)
    # yaw
    sq = 2.0 * (q.w * q.z + q.x * q.y)
    cq = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw =  math.atan2(sq, cq)
    return (roll,pitch,yaw)

def euler2quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x,y,z,w)

class NodoPos(object):
    #%% Inizialization
    def __init__(self):
        self.ekf_pub = rospy.Publisher('/ekf', Odometry, queue_size=10)
        #%% Encoder variables
        self.flag_encR = True
        self.flag_encL = True
        self.wl_ini = 0.0
        self.wr_ini = 0.0
        self.radio = 0.0725
        self.enc_Ts = 0.0
        self.enc_tic =  0.0
        self.wr = 0.0
        self.wl = 0.0
        self.wr_1 = 0.0
        self.wl_1 = 0.0
        self.wr_2 = 0.0
        self.wl_2 = 0.0
        #%% Filter_encoders variables
        self.acce = 20.0
        self.wr_f_2 = 0.0
        self.wr_f_1 = 0.0
        self.wl_f_2 = 0.0
        self.wl_f_1= 0.0
        #%% ICR estimation
        self.ICR = 0.0
        self.ICR_1 = 0.0
        self.ICR_2 = 0.0
        self.ICR_f1 = 0.0
        self.ICR_f2 = 0.0
        #%% IMU variables
        self.flag_imu = True
        self.euler1 = [0,0,0]
        self.imu_toc = 0.0
        self.imu_Ts = 0.0
        self.imu_tic = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        #%% Mag variables
        self.flag_mag = True
        self.yaw_mag1 = 0.0
        self.mag_toc = 0.0
        self.mag_Ts = 0.0
        self.mag_tic = 0.0
        self.mag_yaw = 0.0
        self.mag_yaw_1 = 0.0
        self.Wz = 0.0
        self.x_mag = 0.0
        self.y_mag = 0.0
        self.z_mag = 0.0
        self.Wz_2 = 0.0
        self.Wz_1 = 0.0
        self.Wz_f2 = 0.0
        self.Wz_f1 = 0.0
        
        #%% Kalman variables
        # Matrix error of covariance
        self.kalman_P = np.matrix([[100.0, 0.0, 0.0],[0.0, 100.0, 0.0],[0.0, 0.0, 100.0]])
        # Covariance matrix of process noise
        # self.kalman_Q = np.matrix([[1, 0.0, 0.0],[0.0, 1, 0.0],[0.0, 0.0, 0.001]])
        self.kalman_Q = np.matrix([[5.627197978234252e-07, -8.660344555066894e-08, 1.905427279718744e-06],
                                   [-8.660344555066894e-08, 1.4065639626880477e-08, -2.933464617904681e-07],
                                   [1.905427279718744e-06, -2.933464617904681e-07, 6.460098912945566e-06]])
        # Covariance matrix of measurement noise
        self.kalman_R = 0.0001817647107593486

        

        self.Yg = 1.2
        self.X_hat = np.matrix([[0.0],[0.0],[0.0]])
        #%% INIT
        self.today = date.today()
        self.time = rospy.Time.now()
        print("Starting Kalman node - V2.5.T")
        rospy.Subscriber('/WR', Float32, self.Wr_callbaback)
        rospy.Subscriber('/WL', Float32, self.Wl_callbaback)
        rospy.Subscriber('/imu/data', Imu, self.calIMU)			#100 Hz
        rospy.Subscriber('/imu/mag', MagneticField, self.calMag)	#100 Hz
        
        Flag_out = False
        self.Flag_start = False
        while not Flag_out:
            node_names = rosnode.get_node_names()
            # bag_node = [i for i in node_names if 'play' in i]
            bag_node = [i for i in node_names if 'xsens' in i]
            if (len(bag_node) == 0) and self.Flag_start:
                rospy.signal_shutdown('finish')
                Flag_out = True
            else:
                self.Kalman()
        exit()
    
    #%% Callback Encoders
    def Wl_callbaback(self,data):
        try:
            if self.flag_encL:
                self.wl_2 = data.data
                self.wl_1 = data.data
                #% Displacement of a sample _angular speed_f
                self.wl_f_2 = data.data
                self.wl_f_1 = data.data
                self.wl_ini = data.data
                self.flag_encL = False
            
            self.wl = data.data - self.wl_ini
            # filter Left encoder
            #% Interpolation
            Dwl = 0.08*(self.wl - self.wl_1)/self.enc_Ts
            if (abs(Dwl) > self.acce):
                Delta_wl_1 = self.wl_1 - self.wl_2
                wl = self.wl_1 + Delta_wl_1
            else:
                wl = self.wl
            #% Low-pass filter
            wl_f = 0.02*wl + 0.04*self.wl_1 + 0.02*self.wl_2 - (-1.56)*self.wl_f_1 - 0.64*self.wl_f_2
            #% Displacement of a sample _angular speed
            self.wl_2 = self.wl_1
            self.wl_1 = self.wl
            #% Displacement of a sample _angular speed_f
            self.wl_f_2 = self.wl_f_1
            self.wl_f_1 = wl_f
            #% Output filtered variables
            self.wl = wl_f
        except:
            pass
    
    def Wr_callbaback(self,data):
        # Measurement of sample time of encoders
        self.enc_Ts = rospy.get_time() - self.enc_tic
        self.enc_tic = rospy.get_time()
        
        if self.flag_encR:
            self.wr_2 = data.data
            self.wr_1 = data.data
            #% Displacement of a sample _angular speed_f
            self.wr_f_2 = data.data
            self.wr_f_1 = data.data
            self.wr_ini = data.data
            self.flag_encR = False
            
        self.wr = data.data - self.wr_ini
        # filter Rigth encoder
        #% Interpolation
        Dwr = 0.08*(self.wr - self.wr_1)/self.enc_Ts
        if (abs(Dwr) > self.acce):
            Delta_wr_1 = self.wr_1 - self.wr_2
            wr = self.wr_1 + Delta_wr_1
        else:
            wr = self.wr
        #% Low-pass filter
        wr_f = 0.02*wr + 0.04*self.wr_1 + 0.02*self.wr_2 - (-1.56)*self.wr_f_1 - 0.64*self.wr_f_2
        #% Displacement of a sample _angular speed
        self.wr_2 = self.wr_1
        self.wr_1 = self.wr
        #% Displacement of a sample _angular speed_f
        self.wr_f_2 = self.wr_f_1
        self.wr_f_1 = wr_f
        #% Output filtered variables
        self.wr = wr_f
  
    
    #%% ICR_estimation
    def ICR_estimation(self):
        L = 0.38
        #% ICR calculation
        try:
            ICR = (self.radio/2.0)*(self.wr-self.wl)/self.Wz
            #% Low-pass filter
            self.ICR = 0.00048799*ICR + 0.00097597*self.ICR_1 + 0.00048799*self.ICR_2 - (-1.93655057)*self.ICR_f1 - 0.93850251*self.ICR_f2
            #% Displacement of a sample ICR & _filter
            self.ICR_2 = self.ICR_1
            self.ICR_1 = ICR
            self.ICR_f2 = self.ICR_f1
            self.ICR_f1 = self.ICR
            if self.ICR < L:
                self.ICR = L
        except:
            self.ICR = np.Inf
        
    
    #%% Callback IMU
    def calIMU(self,data):
        self.Flag_start = True
        #% Measurement of sample time of IMU
        self.imu_toc= rospy.get_time()
        self.imu_Ts = (self.imu_toc - self.imu_tic)
        self.imu_tic = rospy.get_time()
        #% Data descomposition
        # qx = data.orientation.x
        # qy = data.orientation.y
        # qz = data.orientation.z
        # qw = data.orientation.w
        # q = np.array([qx,qy,qz,qw])
        # euler = tf.transformations.euler_from_quaternion(q)
        q = data.orientation
        euler = quaternion2euler(q)
        self.imu_pitch = euler[1]
        self.imu_roll  = euler[0]
        #% Quit the initial value of yaw
        if (self.flag_imu):
            self.euler1 = euler
            print('Initial imu data')
            print(' Roll:  %0.6f' % self.euler1[0])
            print(' Pitch: %0.6f' % self.euler1[1])
            print(' yaw:   %0.6f' % self.euler1[2])
            self.flag_imu = False
        self.imu_yaw   = euler[2] - self.euler1[2]
        if (self.imu_yaw < -np.pi):
            self.imu_yaw = self.imu_yaw + 2.0*np.pi
        elif (self.imu_yaw > np.pi):
            self.imu_yaw = self.imu_yaw - 2.0*np.pi
    
      
    #%% Callback Mag
    def calMag(self,data):
        self.mag_yaw_1 = self.mag_yaw
        #% Measurement of sample time of Mag
        self.mag_toc= rospy.get_time()
        self.mag_Ts = (self.mag_toc - self.mag_tic)
        self.mag_tic = rospy.get_time()
        #% Data descomposition
        self.x_mag = data.magnetic_field.x
        self.y_mag = data.magnetic_field.y
        self.z_mag = data.magnetic_field.z
        self.mag_yaw = math.atan2(int(-self.y_mag*1000.0),int(self.x_mag*1000.0))
        #% Quit the initial value of yaw
        if (self.flag_mag):
            self.yaw_mag1 = self.mag_yaw
            print('Initial magnetic yaw: %0.6f' % self.yaw_mag1)
            self.flag_mag = False
        self.mag_yaw   = self.mag_yaw - self.yaw_mag1
        if (self.mag_yaw < -np.pi):
            self.mag_yaw = self.mag_yaw + 2.0*np.pi
        elif (self.mag_yaw > np.pi):
            self.mag_yaw = self.mag_yaw - 2.0*np.pi
        Wz = (self.mag_yaw-self.mag_yaw_1)/self.mag_Ts
        
        #% Low-pass filter
        self.Wz = 0.00048799*Wz + 0.00097597*self.Wz_1 + 0.00048799*self.Wz_2 - (-1.93655057)*self.Wz_f1 - 0.93850251*self.Wz_f2
        #% Displacement of a sample Wz & _filter
        self.Wz_2 = self.Wz_1
        self.Wz_1 = Wz
        self.Wz_f2 = self.Wz_f1
        self.Wz_f1 = self.Wz
  
    
    #%% Callback Kalman
    def Kalman(self):
        if not (self.yaw_mag1==0) and (self.enc_Ts<100):
            self.ICR_estimation()
            U = np.matrix([[self.wr_f_1],[self.wl_f_1]])
            #--------------------------------------------------------------------------#
            #% Modelo diferencial
            A = self.radio * np.array( [[1.0, 1.0] , [1.0/self.ICR, -1.0/self.ICR]] )
            X = 0.5*np.dot(A,U)
            Delta_s = X[0,0]*self.enc_Ts
            Delta_theta = X[1,0]*self.enc_Ts
            # State space
            B = np.array([[Delta_s*np.cos(self.X_hat[2,0]+Delta_theta/2.0)],
                          [Delta_s*np.sin(self.X_hat[2,0]+Delta_theta/2.0)],
                          [Delta_theta]])
            self.X_hat = self.X_hat + B
            if (self.X_hat[2,0] < -3.0*np.pi/2.0):
                self.X_hat[2,0] = self.X_hat[2,0]+2.0*np.pi
            elif (self.X_hat[2,0] > 3.0*np.pi/2.0):
                self.X_hat[2,0] = self.X_hat[2,0]-2.0*np.pi
            # Jacobian computation
            a = -Delta_s*np.sin(self.X_hat[2,0]+Delta_theta/2.0)
            b =  Delta_s*np.cos(self.X_hat[2,0]+Delta_theta/2.0)
            Ak = np.array([[1.0, 0.0, a],[0.0, 1.0, b],[0.0, 0.0, 1.0]])
            # Matrix C
            Ck = np.array([[0.0,0.0,1.0]])
            #% Error covariance matrix
            self.kalman_P =  np.dot(Ak.dot(self.kalman_P), Ak.T) + self.kalman_Q
            #% Kalman filter Gain
            R = np.array(self.kalman_R + np.dot( np.dot(Ck,self.kalman_P) ,Ck.T), dtype='float')
            inv_R = np.linalg.inv( R )
            kalman_K = np.dot(self.kalman_P.dot(Ck.T),inv_R)
            #% update variables
            # State space
            self.X_hat = self.X_hat + kalman_K.dot((self.imu_yaw - self.X_hat[2,0]))
            # Error covariance matrix
            self.kalman_P =  np.dot((np.identity(3) - np.dot(kalman_K,Ck)),self.kalman_P)
            #% Publication
            x = self.X_hat[0,0]
            y = self.X_hat[1,0]
            theta = self.X_hat[2,0]
            self.publish_odom(x, y, theta, Delta_s, Delta_theta)
    
    #%% Callback Publish
    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        # quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        quat = euler2quaternion(0, 0, cur_theta)
        current_time = rospy.Time.now()
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
        self.ekf_pub.publish(odom)
#%% Main
if __name__ == '__main__':
    rospy.init_node("ekf_node")
    cv = NodoPos()
