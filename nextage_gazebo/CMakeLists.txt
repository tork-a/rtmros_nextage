cmake_minimum_required(VERSION 2.8.3)
project(nextage_gazebo)

find_package(catkin REQUIRED COMPONENTS)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nextage_gazebo
#  CATKIN_DEPENDS gazebo_ros gazebo_ros_control nextage_description
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/go_initial.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch test
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/gz.test) # important to have one gz.test files, becuase running gazebo parallely makes trouble on port settings
endif()
