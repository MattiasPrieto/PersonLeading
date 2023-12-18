cmake_minimum_required(VERSION 2.8.3)
project(joy_base)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES joy_base
  CATKIN_DEPENDS
    geometry_msgs
    joy
    roscpp
    sensor_msgs
    std_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)
