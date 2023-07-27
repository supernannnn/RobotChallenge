cmake_minimum_required(VERSION 3.0.2)
project(odom_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(odom_fusion_node
  src/odom_fusion_node.cpp
  src/odom_fusion.cpp
)

target_link_libraries(odom_fusion_node
  ${catkin_LIBRARIES}
)