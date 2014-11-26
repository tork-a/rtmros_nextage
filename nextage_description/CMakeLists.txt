cmake_minimum_required(VERSION 2.8.3)
project(nextage_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

## BEGIN: Moved from nextage_ros_bridge/CMakeLists.txt
# To enable openhrp3
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENHRP3 ${openhrp3_PREFIX}/share/openhrp3)  # for longfloor.wrl
set(COLLADA_EXPORTER ${openhrp3_PREFIX}/share/openhrp3/export-collada)

set(ROBOT_NAME nextage)
message("++ Generate collada file from vrml. nextage_description/models/main.wrl -> ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae")
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  #COMMAND `rospack find openhrp3`/export-collada -i ${PROJECT_SOURCE_DIR}/models/main.wrl -o ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  COMMAND ${COLLADA_EXPORTER} -i ${PROJECT_SOURCE_DIR}/models/main.wrl -o ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  #COMMAND /opt/ros/indigo/share/openhrp3/export-collada -i ${PROJECT_SOURCE_DIR}/models/main.wrl -o ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae  # <-- This passes but not ideal

  DEPENDS ${PROJECT_SOURCE_DIR}/models/main.wrl)
add_custom_target(${PROJECT_NAME}_model_copy ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae)
## END: Moved from nextage_ros_bridge/CMakeLists.txt

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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

## Declare a cpp library
# add_library(nextage_description
#   src/${PROJECT_NAME}/nextage_description.cpp
# )

## Declare a cpp executable
# add_executable(nextage_description_node src/nextage_description_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(nextage_description_node nextage_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(nextage_description_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS nextage_description nextage_description_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Mark cpp header files for installation
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
