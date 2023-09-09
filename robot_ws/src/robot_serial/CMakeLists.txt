cmake_minimum_required(VERSION 3.0.2)
project(robot_serial)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robot_msgs
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS geometry_msgs robot_msgs rospy std_msgs
)

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/rosserial.py
  scripts/position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
