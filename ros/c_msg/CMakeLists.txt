cmake_minimum_required(VERSION 2.8.3)
project(c_msg)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
   FILES
   dummy_encoder.msg
   dummy_IR.msg
   dummy_platform_sensors.msg
   dummy_sonar.msg
   encoder.msg
   IR.msg
   joy.msg
   MotorFeedback.msg
   MotorVel.msg
   platform_sensors.msg
   ServoArray.msg
   sonar.msg
   sonar2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS 
)



