cmake_minimum_required(VERSION 2.8.3)
project(biped_dummy_plug)

find_package(catkin REQUIRED COMPONENTS
  c_msg
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs c_msg)

