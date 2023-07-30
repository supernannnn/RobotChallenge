cmake_minimum_required(VERSION 3.0.2)
project(minimumsnap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  visual_utils
  tf
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS  visual_utils tf
)


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
  src/trajectory_generator_waypoint.cpp
  src/control_cmd_FSM.cpp
)


target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
)