cmake_minimum_required(VERSION 2.8.3)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()
