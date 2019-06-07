# Alpha ROVER
A repository for a 6-wheel rocker bogie rover, based on Robotic Operative System ROS and python to control its perception and action systems.

This repository contents: 
- Source codes 
- Dev scripts

```
/your_root        - path
|--Readme      			 / Instructions to use the AlphaROVER
|--src         			 / scripts for the system
  |--roboclaw_node       /scripts to launch the control of the system
	  |--src        
	  |--config            
	  |--launch with hierarchical scheme
	  |--nodes
	  |--CMakeLists.txt
	  |--LICENCE.md
	  |--package.xml
	  |--README.md
  |--arm.py
  |--maestro.py
  |--com.py
  |--control_mode.py
  |--ipmailer.py
  |--laser_on.py
  |--leds_on.py
  |--lights_off.py
  |--manual_mode.py
  |--rc.local
```

## Hardware Requirements:
- Nvidia Jetson TK1
- Ion Motion Roboclaw, Dual DC motor driver
- Pololu USB servo Control [POL-1353]
- Logitech Wireless Gamepad F710 
- PC host running Ubuntu
- AlphaROVER Robot

## Software Requirements:
- ROS [Kinetic][kin] for Ubuntu 16.04 or ROS [Indigo][ind] for Ubuntu 14.04 on user PC . 
- Python 2.7.15
- RVIZ from ROS
- ROS [Indigo][ind-j] for Ubuntu 14.04 (armhf) on Jetson TK1
- OpenCV

## Getting Started on Jetson TK1
The following steps describe how to configure the Jetson TK1.
- Install [ROS][ind-j]
- Install nano: `sudo apt-get install nano`
- Install Git: `sudo apt-get install git-core`
- Install [Hamachi][ham]
- Remove sudo pass:
`sudo visudo`
```
# Members of the admin group may gain root privileges
%admin ALL=(ALL) ALL
# Allow members of group sudo to execute any command
%sudo   ALL=(ALL:ALL) ALL
```
And replace those lines for these:
```
# Members of the admin group may gain root privileges
%admin ALL=(ALL) NOPASSWD: ALL
# Allow members of group sudo to execute any command
%sudo ALL=(ALL:ALL) NOPASSWD: ALL
```
- USB Rules
Look for your USB devices info:
`udevadm info -a -n /dev/ttyUSB1 | grep`
Create this file with your idVendor and idProduct info:
`sudo nano /etc/udev/rules.d/99-robot.rules`

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", SYMLINK+="tty_roboclaw"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0424", ATTRS{idProduct}=="9514", SYMLINK+="tty_pololu"
```
Save and run:
`sudo udevadm trigger`
`sudo chmod 666 /dev/tty_roboclaw`
`sudo chmod 666 /dev/tty_pololu`

#### Installing USB camera on ROS
- Plug in the USB camera and check if it was recognized by system:
`lsusb `
`ls /dev | grep video*`

- Install usb_cam ROS node:
`sudo apt install ros-indigo-usb-cam`
- Start usb_cam node on slave:
`roslaunch usb_cam usb_cam-test.launch`
- You can read camera data with image_view:
`rosrun image_view image_view image:=/usb_cam/image_raw`
- For web streaming install web-video-server ROS node:
`sudo apt install ros-indigo-web-video-server`
- Create catkin workspace:
`mkdir -p ~/rosvid_ws/src`
`cd ~/rosvid_ws`
`catkin_make`
`source devel/setup.bash`
- Then create ROS package:
`cd src `
`catkin_create_pkg vidsrv std_msgs rospy roscpp `
- Create a launch file:
`mkdir -p vidsrv/launch` 
`nano vidsrv/launch/vidsrv.launch`

And place this code on it:
```
<launch>
  <!-- This node description you can take from usb_cam-test.launch -->
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device" value="/dev/video0" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
  <!-- This node will launch web video server -->
  <node name="web_video_server" pkg="web_video_server" type="web_video_server" />
</launch>
```
- Build package:
`cd ..` 
`catkin_make `
- And run created launch file:
`roslaunch vidsrv vidsrv.launch `
- Now open URL in web browser: {Jetson_IP}:8080
 
#### Functions to add on Jetson's .bashrc file
`nano .bashrc`
```
source /opt/ros/indigo/setup.bash

function control_arm {
  sudo chmod 777 /dev/tty_pololu1
  sudo chmod 777 /dev/tty_pololu2
  sudo chmod 777 /dev/ttyACM1
  python /home/rover/Desktop/Mercury/arm.py &
}


function exportar {
        export ROS_IP = {JetsonIP}
        export ROS_MASTER_URI = http://{pcIP}:11311
}

# If want to connect through hamachi
function exportar_hamachi {
        export ROS_IP = {JetsonHamachiIP}
        export ROS_MASTER_URI=http://{pcHamachiIP}:11311
}

#For local webcam image
function webcam {
        rosrun image_view image_view image:=/usb_cam/image_raw
        #or:
        #rqt_image_view
}

# For web streaming
function webcam_server {
        source ~/rosvid_ws/devel/setup.bash
        roslaunch vidsrv vidsrv.launch
}

function run {
	sudo chmod 777 /dev/tty_roboclaw
	source /home/ubuntu/rbcw_ws/devel/setup.bash
	roslaunch roboclaw_node roboclaw.launch &
	echo $"Run Launched"
}

function kinect {
	roslaunch freenect_launch freenect.launch
}

function ekf {
        source ~/loc_ws/devel/setup.bash
        roslaunch robot_localization ekf_template.launch
}

function imu_node {
	sudo chmod 777 /dev/ttyUSB0
	source /home/ubuntu/imu_ws/devel/setup.bash
	roslaunch xsens_driver xsens.launch
}

# Main function
function rover {
        roscore &
        sleep 4
        piloto &
        run
}

# Add CUDA bin & library paths:
export PATH=/usr/local/cuda/bin:/opt/ros/indigo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games
export LD_LIBRARY_PATH=/usr/local/cuda/lib:/opt/ros/indigo/lib
# Add CUDA bin & library paths:
export PATH=/usr/local/cuda-6.5/bin:/usr/local/cuda/bin:/opt/ros/indigo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games
export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib:/usr/local/cuda/lib:/opt/ros/indigo/lib

sudo python /home/ubuntu/Desktop/Mercury/bash_config.py
```
## Getting Started on host PC
- Install ros according to your ubuntu version, in this case Ubuntu 16.04.
- Install hamachi and haguichi.
#### Configuring a Linux-Supported Joystick with ROS
- Install the package:
`sudo apt-get install ros-kinetic-joy`

- Connect the joystick to your computer and let's see if Linux recognized it:
`ls /dev/input/`
- The joystick will be referred to by jsX, you can test it by running:
`sudo jstest /dev/input/jsX`
Move the joystick around to see the data change. 
- Give permissions on the joystick port:
`sudo chmod a+rw /dev/input/jsX`
- To start the joy node:
`roscore`
`rosparam set joy_node/dev "/dev/input/jsX"`
`rosrun joy joy_node`

To see the data from the joystick:
`rostopic echo joy`

#### Functions to add on PC's .bashrc file
```
source /opt/ros/kinetic/setup.bash

function pilot
{
  ls -l /dev/input/jsX
  sudo chmod a+rw /dev/input/jsX
  rosparam set joy_node/dev "/dev/input/jsX"
  rosrun joy joy_node &
 }

# If want to connect through hamachi
function exportar_hamachi {
        export ROS_IP = {pcIP}
}

function exportar
{
  export ROS_IP = {pcIP}
}

function arm
{
  rostopic echo arm_string
}

function joy_prove
{
  ls -l /dev/input/js1
  sudo chmod a+rw /dev/input/js1

  rosparam set joy_node/dev "/dev/input/js1"
  rosrun joy joy_node &
  rostopic echo joy
}

# For local Jetson webcam image, 
# It presents more latency than the web streaming method (webcam_server)
function webcam
{
	rosrun image_view image_view image:=/usb_cam/image_raw &
	#or:
	#rqt_image_view
}
```



## Authors:
**Universidad de Ibagué** - **Ingeniería Electrónica.**
**Proyecto de Grado 2019/A**
- [Nickson E. Garcia](mailto:nicksongarcia@ieee.org)
- [Cristian G. Molina](mailto:2420132009@estudiantesunibague.edu.co) 
- [Harold F. Murcia](www.haroldmurcia.com)
***
