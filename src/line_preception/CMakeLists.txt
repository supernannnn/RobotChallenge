cmake_minimum_required(VERSION 3.0.2)
project(line_preception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include

)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS} 
)


add_executable(line_preception_node
  src/line_preception_node.cpp 
  )

target_link_libraries(line_preception_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
