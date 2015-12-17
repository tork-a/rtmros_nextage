cmake_minimum_required(VERSION 2.8.3)
project(nextage_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
)

## BEGIN: Moved from nextage_ros_bridge/CMakeLists.txt
# To enable openhrp3
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENHRP3 ${openhrp3_PREFIX}/share/openhrp3)  # for longfloor.wrl
set(COLLADA_EXPORTER ${openhrp3_PREFIX}/share/openhrp3/export-collada)

set(ROBOT_NAME main)  # Aiming main.dae. Use same file prefix with VRML for intuitition
message("++ Generate collada file from vrml. nextage_description/models/main.wrl -> ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae")
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  COMMAND ${COLLADA_EXPORTER} -i ${PROJECT_SOURCE_DIR}/models/main.wrl -o ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  DEPENDS ${PROJECT_SOURCE_DIR}/models/main.wrl)
add_custom_target(${PROJECT_NAME}_model_copy ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae)
## END: Moved from nextage_ros_bridge/CMakeLists.txt

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nextage_description
#  CATKIN_DEPENDS xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY www    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nextage_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
