cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

## Use C++14
add_compile_options(-std=c++14)

## Find catkin macros and ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  moveit_ros_planning_interface
)

## Catkin package
catkin_package(
  CATKIN_DEPENDS roscpp tf geometry_msgs moveit_ros_planning_interface
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Node: move_above_object_node
add_executable(move_above_object_node
  src/arm_control_moveit.cpp
)

target_link_libraries(move_above_object_node
  ${catkin_LIBRARIES}
)

## Install
install(TARGETS
  move_above_object_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
