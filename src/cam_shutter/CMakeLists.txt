cmake_minimum_required(VERSION 3.0.2)
project(cam_shutter)


find_package(catkin REQUIRED COMPONENTS roslaunch
  roscpp
  rospy
  std_msgs
  std_srvs
)
roslaunch_add_file_check(launch)


catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/cam_shutter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
