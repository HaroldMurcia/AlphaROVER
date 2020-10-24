#!/bin/bash
# -*- ENCODING: UTF-8 -*-

echo "
==============================================================================
Title           :alphaROVER.sh
Description     :This script manages the alphaROVER.
Author          :Harold F MURCIA - www.haroldmurcia.com
Date            :16/09/2020
Version         :0.1
Usage           :source ~/alphaROVER.sh
Notes           :Calling from .bashrc
==============================================================================
"

today=$(date +%Y%m%d)
echo $today
div=======================================

printf "
      _       _         _____  _____  _____  _____  _____
 ___ | | ___ | |_  ___ | __  ||     ||  |  ||   __|| __  |
| .'|| || . ||   || .'||    -||  |  ||  |  ||   __||    -|
|__,||_||  _||_|_||__,||__|__||_____| \___/ |_____||__|__|
        |_|
"
# Paths
cd ~/catkin_ws/src/AlphaROVER/src/Config
path_alpha_config=$(pwd)
cd ../..
path_alphaROVER=$(pwd)
cd
echo "PATH:"$path_alphaROVER

# Variables
hokuyo_ip="192.168.0.10"

# Instal mit_node
DIR=$path_alpha_config$'/xsens_mt'
if [ -d "$DIR" ]; then
  ### Take action if $DIR exists ###
  cd ~/catkin_ws
  catkin_make
  sleep 10
  cd
  echo "mti config done!"
else
  cd $path_alpha_config
  git clone https://github.com/xsens/xsens_mt.git
  cd xsens_mt
  make
  sleep 10
  cd ~/catkin_ws
  catkin_make
  sleep 10
  cd
  ### echo "Error: ${DIR} not found. Can not continue."
fi

# Ports
sudo chmod -R 777 /dev/tty_roboclaw
#sudo chmod -R 777 /dev/tty_Arduino
sudo chmod -R 777 /dev/tty_Dynamixel
#sudo chmod -R 777 /dev/tty_pololu
sudo chmod -R 777 /dev/tty_GPS
sudo chmod -R 777 /dev/tty_um7

# GPIOS Jetson NANO
# 1: 1.6 volts	0:Zero volts
LEDS="76"	#GPIO 76 is pin 35 --> LEDS
LASER_A="12"	#GPIO 12 is pin 37 --> LASER
LASER_B="38"	#GPIO 38 is pin 33 --> LASER
echo $LEDS > /sys/class/gpio/export
echo $LASER_A > /sys/class/gpio/export
echo $LASER_B > /sys/class/gpio/export

sleep 3

echo out > /sys/class/gpio/gpio$LEDS$"/direction"
echo out > /sys/class/gpio/gpio$LASER_A$"/direction"
echo out > /sys/class/gpio/gpio$LASER_B$"/direction"


# Functions
function click {
  sleep 0.2
	echo 1 > /sys/class/gpio/gpio$LEDS$"/value"
	echo 1 > /sys/class/gpio/gpio$LASER_A$"/value"
	echo 1 > /sys/class/gpio/gpio$LASER_B$"/value"
	sleep 0.1
	echo 0 > /sys/class/gpio/gpio$LEDS$"/value"
	echo 0 > /sys/class/gpio/gpio$LASER_A$"/value"
	echo 0 > /sys/class/gpio/gpio$LASER_B$"/value"
	sleep 0.1
	echo 1 > /sys/class/gpio/gpio$LEDS$"/value"
	echo 1 > /sys/class/gpio/gpio$LASER_A$"/value"
	echo 1 > /sys/class/gpio/gpio$LASER_B$"/value"
	sleep 0.1
	echo 0 > /sys/class/gpio/gpio$LEDS$"/value"
	echo 0 > /sys/class/gpio/gpio$LASER_A$"/value"
	echo 0 > /sys/class/gpio/gpio$LASER_B$"/value"
}

click

function urg_node {
	echo "======================================="
	rosrun urg_node urg_node _ip_address:=$hokuyo_ip _publish_multiecho:="true" &
	sleep 1
	printf "Hokuyo ready...\n"
}

function dynamixel_node {
	echo "======================================="
	ls -l /dev/tty_Dynamixel
	roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch usb_port:=/dev/tty_Dynamixel &
	sleep 1
	roslaunch dynamixel_workbench_operators joint_operator.launch &
	sleep 1
	roslaunch dynamixel_move dynamixel_move.launch &
	sleep 1
	printf "Dynamixel ready...\n"
}

function xsens_node {
	echo "======================================="
	sudo modprobe usbserial
	sudo insmod $path_alpha_config$"/xsens_mt/xsens_mt.ko"
	sleep 1
	sudo chmod -R 777 /dev/tty_imu
	ls -l /dev/tty_imu
	roslaunch xsens_driver xsens.launch  &
	printf $"Xsens MTi-10 ready...\n"
}

function kinect_node {
	echo "======================================="
	roslaunch freenect_launch freenect.launch depth_registration:=true data_skip:=18 &
	sleep 3
	echo $"Kinect v1 ready...\n"
}

function webcam {
	echo "======================================="
  roslaunch usb_cam usb_cam-test.launch &
	sleep 5
	printf $"Main camera ready...\n"
}

function roboclaw_node {
	echo "======================================="
	ls -l /dev/tty_roboclaw
	roslaunch roboclaw_node roboclaw.launch &
	sleep 3
	printf "Roboclaw ready...\n"
}

function arm_node {
	echo "======================================="
	python $path_alphaROVER$"/src/Arm/arm.py" &
	sleep 1
	printf "Arm ready...\n"
}

function ekf_node {
	echo "======================================="
	python $path_alphaROVER$"/src/EKF/ekf2.py" &
	printf "EKF ready...\n"
}

function gscamInit_node {
	echo "======================================="
	export GSCAM_CONFIG="v4l2src device=/dev/video1 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
	rosrun gscam gscam &
	printf "gscam is ready...\n"
}

function pilot {
	echo "======================================="
	ls -l /dev/input/js0							  # check joystick connection
	sudo chmod a+rw /dev/input/js0			# permission options (all) + (read)(write)
	rosparam set joy_node/dev "/dev/input/js0"	# ROS parameter assignment
	rosrun joy joy_node &								# Run Joy_node
	printf "Joy is ready...\n"
}

function USB {
	echo "======================================="
	ls -l /dev/sda1
	sudo mount -t vfat /dev/sda1 /media/usb/ -o uid=1000,gid=1000
	cd /media/usb
	printf "USB is ready...\n"
}

function exportar {
  export ROS_IP=alpha-rover.local
	export ROS_MASTER_URI=http://sebas-Ubuntu.local:11311
}

function exportar_ws {
        export ROS_IP=alpha-rover.local
        export ROS_MASTER_URI=http://UILABAUT5820.local:11311
}

function gps_node {
        echo "======================================="
	ls -l /dev/tty_GPS
	roslaunch lea_6h_gps lea_6h_gps.launch device_name:=/dev/tty_GPS &
	printf "GPS is ready...\n"
}

function um7_node {
        echo "======================================="
	chmod +x $path_alphaROVER$"/src/um7_node/src/um7_node.py"
	ls -l /dev/tty_um7
	roslaunch um7_node um7_node.launch device_name:=/dev/tty_um7 &
	printf "UM7 imu is ready...\n"
}

function exportar_hamachi {
        export ROS_IP=25.101.143.22
        export ROS_MASTER_URI=http://25.103.174.150:11311
}

function Alpha_update {
        git clone https://github.com/Tilaguy/AlphaROVER.git
}

function rover {
	printf "\n======================================="
	roscore &
	sleep 4
	pilot &
	sleep 2
	xsens_node &
	sleep 2
	roboclaw_node &
	sleep 3
	ekf_node &
	sleep 7
	printf $"ROVER READY\n"
	printf "=======================================\n"
}
