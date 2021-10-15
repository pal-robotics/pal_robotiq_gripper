cmake_minimum_required(VERSION 2.8.3)
project(pal_robotiq_gripper_wrapper_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  # other_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_message_files(
  FILES
  GripperStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  # other_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

