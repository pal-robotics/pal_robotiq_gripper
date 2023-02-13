cmake_minimum_required(VERSION 2.8.3)
project(pal_robotiq_gripper_wrapper)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  rospy
  trajectory_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pal_robotiq_gripper_wrapper
#  CATKIN_DEPENDS actionlib control_msgs rospy trajectory_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

install(PROGRAMS
  scripts/gripper_grasping.py
  scripts/gripper_epick_grasping.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
