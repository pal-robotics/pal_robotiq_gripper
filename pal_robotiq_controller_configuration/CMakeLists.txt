cmake_minimum_required(VERSION 2.8.3)
project(pal_robotiq_controller_configuration)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if (BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

# foreach(dir config launch)
#     install(DIRECTORY ${dir}/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

# install(PROGRAMS scripts/home_gripper.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

