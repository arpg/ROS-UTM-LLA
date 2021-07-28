cmake_minimum_required(VERSION 2.8.3)
project(utm_lla)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

#test on 1.34
find_package (GeographicLib REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES utm2lla
#  CATKIN_DEPENDS deepmap_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(utm_lla_converter src/convert_ndt2lla.cpp src/utm_lla_converter.cpp)
target_link_libraries(utm_lla_converter ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})

