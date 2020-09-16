# Alpha ROVER
A repository for a 6-wheel rocker bogie rover, based on Robotic Operative System ROS and Python to control the perception and action systems.

* Watch the YouTube video [here](https://www.youtube.com/watch?v=stQqyb9inXY&t=333s)

This repository contents:
- Source codes
- Dev scripts
- Data files

```
/your_root        - path
|--Readme      			 / Instructions to use the AlphaROVER
|--src         			 / scripts for the system
  |--alpha_pc            / scripts to launch the control of the system
	  |--arm.py
	  |--maestro.py
	  |--com.py
	  |--control_mode.py
	  |--ekf.py
	  |--ipmailer.py
	  |--laser_on.py
	  |--leds_on.py
	  |--lights_off.py
	  |--manual_mode.py
	  |--rc.local
	  |--roboclaw_ros	/ modified from: https://github.com/sonyccd/roboclaw_ros.git
		  |--LICENCE.md
		  |--README.md
		  |--roboclaw_node
  |--user_pc
	  |--cam_bridge.py
|--data
	|--mechanics  
		|--alpha_full.pdf
		|--inventor_files
		|--photos
	|--electronics
		|--diagrams
		|--proteus_files      			 
```

## Jetson TK1
Based on: https://developer.download.nvidia.com/embedded/L4T/r21_Release_v8.0/release_files/l4t_quick_start_guide.txt?2AJGr6ik6_g9nwN5HQ5Q2Zh8DJOTP4hQvehjztkoRcC0o4x_35fPIgZ8OOStLerVgY6a37NIBN9VFDC-kZmbKVFt7QG3zCIAF565Ages-cXgJkXhkqV_fGzqVarbLOQ-JTuidCfq2qnsmdUVltVu4ifN7R8m6NfQpobz6UxapI47LlCS
```
mkdir LT4
cd LT4
wget https://developer.nvidia.com/embedded/dlc/tk1-driver-package-r218
wget https://developer.nvidia.com/embedded/dlc/sample-root-filesystem-r218
sudo tar xpf Tegra124_Linux_R21.8.0_armhf.tbz2
cd Linux_for_Tegra/rootfs
sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_R21.8.0_armhf.tbz2
cd ..
sudo ./apply_binaries.sh
```
Put Jetson TK1 into “recovery mode” by holding down the RECOVERY button while pressing and releasing the RESET button once on the main board.
```
sudo ./flash.sh jetson-tk1 mmcblk0p1
```

## Jetson nano
Follow the isntructions from: [NVIDIA](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write)
* Download the las SD card [image](https://developer.nvidia.com/jetson-nano-sd-card-image)
* Insert the SD and open the temrinal application
* dmesg | tail | awk '$3 == "sd" {print}'
* /usr/bin/unzip -p ~/Downloads/jetson_nano_devkit_sd_card.zip | sudo /bin/dd of=/dev/sd<x> bs=1M status=progress
* sudo eject /dev/sd<x>
```
$ rosdep apt autoremove thunderbird*
$ sudo apt autoremove libreoffice*
$ sudo apt-get remove --purge libreoffice*
sudo apt-get remove libreoffice-core
sudo apt-get remove snapd lightdm cups chromium*
sudo apt-get remove libcurlpp0
$ sudo apt autoremove transmission-gtk
sudo systemctl set-default multi-user.target
$ rosdep update
$ rosdep upgrade
$sudo apt install htop
sudo apt install screen
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
#sudo apt install ros-melodic-desktop
$ sudo apt install ros-melodic-ros-base
$ apt search ros-melodic
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo rosdep init
$ rosdep update
```
sudo apt-get install nano

### Catkin configuration
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

## Hardware Requirements:
- Nvidia Jetson TK1 or PC with Linux
- Ion Motion Roboclaw, Dual DC motor driver
- Pololu USB servo Control [POL-1353]
- Logitech Wireless Gamepad F710
- PC host running Ubuntu
- AlphaROVER platform

## Software Requirements:
- ROS [Kinetic][kin] for Ubuntu 16.04 or ROS [Indigo][ind] for Ubuntu 14.04 on user PC .
- Python 2.7.15
- RVIZ from ROS
- ROS [Indigo][ind-j] for Ubuntu 14.04 (armhf) on Jetson TK1
- ROS [Melodic] for Ubuntu 18.04 (armhf) on Jetson NANO
- OpenCV
- Hamachi
- Modified node *xsens_mti_ros_node* available [here.](https://github.com/HaroldMurcia/xsens_mti_ros_node)

#### Installing USB camera on ROS Jetson TK1
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

**Notes/**
* *Paths and code of roboclaw_node.py must be updated according to the pc installation.*
* `~/AlphaROVER/src/alpha_pc/rc.local` must be located on `/etc` folder.

`nano .bashrc`
```
source /opt/ros/indigo/setup.bash

function arm {
        sudo chmod 777 /dev/tty_pololu
        python ~/AlphaROVER/src/alpha_pc/arm.py
        sleep 1
        echo $"Arm OK..."
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

#Launch main webcam
function webcam {
	roslaunch usb_cam usb_cam-test.launch &
        sleep 5
        echo $"Main camera ready..."
}

function run {
        sudo chmod 777 /dev/tty_roboclaw
        source /home/ubuntu/rbcw_ws/devel/setup.bash
        roslaunch roboclaw_node roboclaw.launch &
        sleep 3
        echo $"Run Launched"
}

function kinect {
	roslaunch freenect_launch freenect.launch
}

function ekf {
        python ~/AlphaROVER/src/alpha_pc/ekf.py
}

function imu_node {
	sudo chmod 777 /dev/ttyUSB0
	source /home/ubuntu/xsens_ws/devel/setup.bash
	roslaunch xsens_driver xsens.launch
}


# Main function
function rover {
        roscore &
        sleep 4
        piloto &
        echo $"Launched piloto"
        sleep 2
        run &
        sleep 2
        echo $"Ready..."
}

sudo python ~/AlphaROVER/src/alpha_pc/bash_config.py
```

## Getting Started on host PC
- Install ros according to your ubuntu version, in this case Ubuntu 16.04.
- Install hamachi and haguichi.
### Configuring a Linux-Supported Joystick with ROS
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

### Functions to add on PC's .bashrc file
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
function exportar_hamachi
{
        export ROS_IP = {pcIP}
}

function exportar
{
  export ROS_IP = {pcIP}
}

# To access Jetson webcam image
function webcam
{
	python ~/cam_bridge.py
}
```



## Authors:
**[Universidad de Ibagué - Ingeniería Electrónica.](https://electronica.unibague.edu.co)**
**Proyecto de Grado 2019/A**
- [Nickson E. Garcia](mailto:nicksongarcia@ieee.org)
- [Cristian G. Molina](mailto:2420132009@estudiantesunibague.edu.co)
- [Harold F. Murcia](www.haroldmurcia.com)
***

[ros]: <http://www.ros.org/>
[lib-rc]: <http://www.basicmicro.com/downloads>
[pol]: <https://www.pololu.com/product/1353>
[jet]: <https://developer.nvidia.com/embedded/downloads#?tx=$product,jetson_tk1$software,l4t-tk1>
[kin]: <http://wiki.ros.org/kinetic/Installation/Ubuntu>
[ind]: <http://wiki.ros.org/indigo/Installation/Ubuntu>
[ind-j]: <http://wiki.ros.org/indigo/Installation/UbuntuARM>
[ham]: <https://medium.com/@KyleARector/logmein-hamachi-on-raspberry-pi-ad2ba3619f3a>
