cmake_minimum_required(VERSION 2.8.3)
project(pal_robotiq_gripper_wrapper)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# # include_directories(include)
# include_directories(
#   SYSTEM
#   ${catkin_INCLUDE_DIRS}
# )


# #############
# ## Install ##
# #############

# install(PROGRAMS
#   scripts/gripper_grasping.py
#   scripts/gripper_epick_grasping.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# foreach (dir launch config)
#   install(DIRECTORY ${dir}/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()