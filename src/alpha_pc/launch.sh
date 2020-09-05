#!/bin/bash
# -*- ENCODING: UTF-8 -*-


function arm {
	#sudo chmod 777 /dev/tty_pololu1
	#sudo chmod 777 /dev/tty_pololu2
	sudo chmod 777 /dev/ttyACM0
	sudo chmod 777 /dev/ttyACM1
	sudo chmod 777 /dev/ttyACM2
	python /home/ubuntu/Desktop/Mercury/arm.py
	sleep 1
	echo $"Arm OK..."
}

function urg_node {
	rosrun urg_node urg_node _ip_address:="192.168.0.10" _publish_multiecho:="true" &
	sleep 3
}

function dynamixel_node {
        roslaunch dynamixel_controllers controller_manager.launch &
	sleep 3
	roslaunch dynamixel_controllers start_tilt_controller.launch
	sleep 1
	echo $"Dynamixel OK"
}

function exportar {
        export ROS_IP=tegra-ubuntu.local
	export ROS_MASTER_URI=http://sebas-Ubuntu.local:11311
}

function exportar_ws {
        export ROS_IP=tegra-ubuntu.local
        export ROS_MASTER_URI=http://UILABAUT5820.local:11311
}

function exportar_hamachi {
        export ROS_IP=25.101.143.22
        export ROS_MASTER_URI=http://25.103.174.150:11311
}

function webcam {
        roslaunch usb_cam usb_cam-test.launch &
	sleep 5
	echo $"Main camera ready..."
}

function imu_node {
	sudo chmod 777 /dev/tty_imu
	source /home/ubuntu/xsens_ws/devel/setup.bash
	roslaunch xsens_driver xsens.launch  &
}


function piloto {
	ls -l /dev/input/js0				# check joystick connection
	sudo chmod a+rw /dev/input/js0			# permission options (all) + (read)(write)
	rosparam set joy_node/dev "/dev/input/js0"	# ROS parameter assignment
	rosrun joy joy_node &				# Run Joy_node
}

function run {
	sudo chmod 777 /dev/tty_roboclaw
	source /home/ubuntu/rbcw_ws/devel/setup.bash
	roslaunch roboclaw_node roboclaw.launch &
	sleep 3
	echo $"Run Launched"				# print in console
}

function move {
        sudo chmod 777 /dev/tty_roboclaw
        source /home/ubuntu/move_ws/devel/setup.bash
        roslaunch roboclaw_node roboclaw.launch &
        echo $"Move Launched"
}

function kinect {
	roslaunch freenect_launch freenect.launch depth_registration:=true data_skip:=18 &
	#roslaunch freenect_launch freenect.launch
	sleep 3
}

function idinput {
        sudo chmod 777 /dev/tty_roboclaw
        source /home/ubuntu/idinput_ws/devel/setup.bash
        roslaunch roboclaw_node roboclaw.launch &
        rosbag record -a --duration=70 -O idinput_tierra_15hz.bag
	echo $"idinput Launched"
}

#function scan_node {
#	 python Desktop/Mercury/ini_scan.py & 
#	 echo $"Scan manager OK"
#}

function ekf {
        #python ~/Desktop/Mercury/pose_imu_ang.py &
	python ~/Desktop/Mercury/ekf2.py &
}

function remote_rover {
        #exportar_hamachi
	roscore &
        sleep 4
        run &
	sleep 3
        echo $"Ready..."
}

function GPS_launch {
	sudo chmod 777 /dev/tty_Arduino
	python ~/Desktop/Mercury/Arduino_serial.py &
	sleep 2
}

function rover {
	roscore &
	sleep 4
	piloto &
	sleep 2
	imu_node &
	sleep 2
	run &
	sleep 3
	ekf &
	sleep 7
	echo $"EKF launched"
#	GPS_launch &
#	sleep 3
#	echo "Arduino connected"
#	dynamixel_node
#	sleep 2
#	urg_node
#	sleep 1
#	kinect
#	sleep 10
#	#scan_node
	echo $"Ready..."
}

function save_data {
	echo "file name: $1"
	#rosbag record -o ".bag" /echoes /IMU_LiDAR /camera/rgb/image_color /tilt_controller/state /tilt_controller/command /GPS /RefL /RefR /WL /WR /ekf /encoders /imu_data /joy /mti/sensor/imu_free /mti/sensor/magnetic /odom /pid
}

function gscamInit()
{
	export GSCAM_CONFIG="v4l2src device=/dev/video1 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
	rosrun gscam gscam &
	echo "gscam is ready"
}

function USB {
	sudo mount -t vfat /dev/sda1 /media/usb/ -o uid=1000,gid=1000
	cd /media/usb
	echo "USB"
}

sudo python /home/ubuntu/Desktop/Mercury/bash_config.py
