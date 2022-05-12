cmake_minimum_required(VERSION 3.0.2)
project(digit_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Digit_Keyboard_Standing_Gains.msg
  Digit_Keyboard_Walking_Targets.msg
  Digit_Json_Info.msg
  Digit_Gamepad_Walking_Targets.msg
  Digit_Keyboard_Standing_Targets.msg
  Digit_Gamepad_Standing_Targets.msg
  Digit_Observation.msg
  Digit_Commands.msg)

add_service_files(
  FILES 
  Digit_Observation_srv.srv 
  Digit_Commands_srv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

