cmake_minimum_required(VERSION 2.8.3)
project(nextage_ik_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME_L nextage_left_arm_moveit_ikfast_plugin)
set(IKFAST_LIBRARY_NAME_R nextage_right_arm_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME_L} src/nextage_left_arm_ikfast_moveit_plugin.cpp)
add_library(${IKFAST_LIBRARY_NAME_R} src/nextage_right_arm_ikfast_moveit_plugin.cpp)

find_package(PkgConfig)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
if ("${URDFDOM_VERSION}" VERSION_EQUAL "1.0.0" OR "${URDFDOM_VERSION}" VERSION_GREATER "1.0.0")
  add_definitions(-DURDFDOM_1_0_0_API)
endif()

target_link_libraries(${IKFAST_LIBRARY_NAME_L} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
target_link_libraries(${IKFAST_LIBRARY_NAME_R} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
if ($ENV{ROS_DISTRO} STRGREATER "indigo" AND $ENV{ROS_DISTRO} STRLESS "melodic")  # On melodic, default is C++14, so the following is not needed
  target_compile_features(${IKFAST_LIBRARY_NAME_L} PRIVATE cxx_range_for)
  target_compile_features(${IKFAST_LIBRARY_NAME_R} PRIVATE cxx_range_for)
endif()

install(TARGETS ${IKFAST_LIBRARY_NAME_L} ${IKFAST_LIBRARY_NAME_R} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  nextage_left_arm_moveit_ikfast_plugin_description.xml
  nextage_right_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
