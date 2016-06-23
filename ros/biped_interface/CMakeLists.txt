cmake_minimum_required(VERSION 2.8.3)
project(biped_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ServoArray.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    sensor_msgs
    tf
    message_runtime
  DEPENDS
    python_orocos_kdl
)



