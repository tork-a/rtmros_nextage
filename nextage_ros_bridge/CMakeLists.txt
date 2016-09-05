cmake_minimum_required(VERSION 2.8.3)
project(nextage_ros_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hironx_ros_bridge nextage_description roslint rostest)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nextage_ros_bridge
  CATKIN_DEPENDS hironx_ros_bridge nextage_description
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_python()

###################################################
## Generate Roobt Models and Configuration Files ##
###################################################

# set ROBOT_NAME and OPENHRP3 for configure_file
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
set(OPENHRP3 ${openhrp3_PREFIX}/share/openhrp3)  # for longfloor.wrl
set(ROBOT_NAME nextage)

message("hironx_ros_bridge_SOURCE_DIR => ${hironx_ros_bridge_SOURCE_DIR}")
message("hironx_ros_bridge_SOURCE_PREFIX => ${hironx_ros_bridge_SOURCE_PREFIX}")
message("hironx_ros_bridge_PREFIX => ${hironx_ros_bridge_PREFIX}")

if(NOT EXISTS ${hironx_ros_bridge_SOURCE_DIR}/conf)
  if (EXISTS ${hironx_ros_bridge_SOURCE_PREFIX}/conf)
    set(hironx_ros_bridge_SOURCE_DIR ${hironx_ros_bridge_SOURCE_PREFIX})
  else(EXISTS ${hironx_ros_bridge_SOURCE_PREFIX}/conf)
    set(hironx_ros_bridge_SOURCE_DIR ${hironx_ros_bridge_PREFIX}/share/hironx_ros_bridge)
  endif(EXISTS ${hironx_ros_bridge_SOURCE_PREFIX}/conf)
endif()

file(MAKE_DIRECTORY conf)
configure_file(${hironx_ros_bridge_SOURCE_DIR}/conf/RobotHardware.conf.in       ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.RobotHardware.conf)
configure_file(${hironx_ros_bridge_SOURCE_DIR}/conf/nosim.RobotHardware.conf.in ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.RobotHardware.conf)
configure_file(${hironx_ros_bridge_SOURCE_DIR}/conf/xml.in                      ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml)
configure_file(${hironx_ros_bridge_SOURCE_DIR}/conf/nosim.xml.in                ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml)
configure_file(${hironx_ros_bridge_SOURCE_DIR}/conf/conf.in                     ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.conf)
configure_file(${hironx_ros_bridge_SOURCE_DIR}/conf/nosim.conf.in               ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.conf)
add_custom_target(${PROJECT_NAME}_model_files ALL DEPENDS ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.RobotHardware.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.RobotHardware.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.conf ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.conf
  COMMAND sed -i 's@property name="WAIST.translation" value="0  0  -0.1"@property name="WAIST.translation" value="0  0  -1.03"@' ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml
  COMMAND sed -i 's@property name="WAIST.translation" value="0  0  -0.1"@property name="WAIST.translation" value="0  0  -1.03"@' ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml
  COMMAND sed -i 's@property name="eyeHomePosition" value="-0.70711 -0 0.70711 2 0.70711 -0 0.70711 2 0 1 0 0.8 0 0 0 1 "@property name="eyeHomePosition" value="-0.707107 -0.183013 0.683013 2 0.707107 -0.183013 0.683013 2 0 0.965926 0.258819 0.8 "@' ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml
  COMMAND sed -i 's@property name="eyeHomePosition" value="-0.70711 -0 0.70711 2 0.70711 -0 0.70711 2 0 1 0 0.8 0 0 0 1 "@property name="eyeHomePosition" value="-0.707107 -0.183013 0.683013 2 0.707107 -0.183013 0.683013 2 0 0.965926 0.258819 0.8 "@' ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml
  COMMAND sed -i 's@property name="showScale" value="true"@property name="showScale" value="false"@' ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}.xml
  COMMAND sed -i 's@property name="showScale" value="true"@property name="showScale" value="false"@' ${PROJECT_SOURCE_DIR}/conf/${ROBOT_NAME}_nosim.xml
  )


#compile_openhrp_model(${nextage_description_SOURCE_DIR}/models/main.wrl)
message("++ nextage_description/models/main.dae -> ${nextage_description_SOURCE_DIR}/models/main.dae")
if(NOT EXISTS ${nextage_description_SOURCE_DIR}/models/main.dae)
  if(EXISTS ${nextage_description_SOURCE_PREFIX}/models/main.dae)
    set(nextage_description_SOURCE_DIR ${nextage_description_SOURCE_PREFIX})
  else(EXISTS ${nextage_description_SOURCE_PREFIX}/models/main.dae)
    set(nextage_description_SOURCE_DIR ${nextage_description_PREFIX}/share/nextage_description)
  endif(EXISTS ${nextage_description_SOURCE_PREFIX}/models/main.dae)
endif()
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  COMMAND ${CMAKE_COMMAND} -E copy ${nextage_description_SOURCE_DIR}/models/main.dae ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae
  DEPENDS ${nextage_description_SOURCE_DIR}/models/main.dae)
add_custom_target(${PROJECT_NAME}_model_copy ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/${ROBOT_NAME}.dae)

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
# install(TARGETS nextage_ros_bridge nextage_ros_bridge_node
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
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY script DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY conf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN "*.in" EXCLUDE)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

install(CODE "
  message(\"++ glob files under \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/conf/\")
  file(GLOB _xml_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/conf/*.xml)
  file(GLOB _conf_files \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/conf/*.conf)
  foreach(_file \${_xml_files};\${_conf_files})
    message(\"++ sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
    message(\"sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file}\")
    execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}@ \${_file})
  endforeach()
  ")

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nextage_ros_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

add_rostest(test/nxo.test)
