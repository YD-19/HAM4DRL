cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_ddpg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_ddpg
  CATKIN_DEPENDS gazebo_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
