cmake_minimum_required(VERSION 2.8.3)
project(nextage_moveit_config)

find_package(catkin REQUIRED rostest)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

# installing .desktop into ~/Desktop
set(SETUP_DIRECTORY $ENV{XDG_DESKTOP_DIR})
if ("${SETUP_DIRECTORY}" STREQUAL "")
  set(SETUP_DIRECTORY $ENV{HOME}/Desktop)
endif ("${SETUP_DIRECTORY}" STREQUAL "")

set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR})
configure_file(${PROJECT_SOURCE_DIR}/NextageROS.desktop.in ${SETUP_DIRECTORY}/NextageROS.desktop)
add_custom_target(desktop ALL DEPENDS ${PROJECT_SOURCE_DIR}/NextageROS.desktop.in)

add_rostest(test/joy.test)
add_rostest(test/nxo_moveit.test)
