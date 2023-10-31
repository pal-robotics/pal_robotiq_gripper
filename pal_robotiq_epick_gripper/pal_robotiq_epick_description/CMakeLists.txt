cmake_minimum_required(VERSION 3.0.2)
project(pal_robotiq_epick_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if (BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)

# foreach(dir meshes robots urdf)
#     install(DIRECTORY ${dir}/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)
