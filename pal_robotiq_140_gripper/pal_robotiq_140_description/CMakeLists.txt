cmake_minimum_required(VERSION 3.0.2)
project(pal_robotiq_140_description)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pal_robotiq_140_description
#  CATKIN_DEPENDS xacro
#  DEPENDS system_lib
)

foreach(dir meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
