cmake_minimum_required(VERSION 2.8.3)
project(nextage_calibration)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nextage_calibration
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

install(DIRECTORY conf launch models test resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/gz_chest.test)
endif()
