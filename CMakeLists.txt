cmake_minimum_required(VERSION 2.8.3)

project(serial_arm_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  #std_msgs
  urdf
  kdl_parser
  #roslib
  #orocos_kdl
)

#find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serialArmKin
  CATKIN_DEPENDS roscpp #urdf
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# Define a library target called serialArmKin
add_library(serialArmKin src/serialArmKin.cpp)
target_link_libraries(serialArmKin ${catkin_LIBRARIES})

# Define a library target called errorMsg
#add_library(errorMsg src/errorMsg.cpp)
#target_link_libraries(errorMsg ${catkin_LIBRARIES})
