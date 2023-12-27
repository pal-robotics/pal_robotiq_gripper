cmake_minimum_required(VERSION 3.5)
project(pal_robotiq_controller_configuration)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS scripts/home_gripper.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch)
