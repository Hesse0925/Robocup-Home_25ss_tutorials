cmake_minimum_required(VERSION 3.0.2)
project(tiago_move)

## Compile as C++11 (recommended)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  moveit_ros_planning_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib move_base_msgs roscpp moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 1. 原节点：tiago_move_node
add_executable(${PROJECT_NAME}_node src/tiago_move_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## 2. 原节点：run_traj_control
add_executable(run_traj_control src/run_traj_control.cpp)
target_link_libraries(run_traj_control ${catkin_LIBRARIES})

## 3. 新增节点：move_right_node
add_executable(move_right_node src/no_cup_react_node.cpp)
target_link_libraries(move_right_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_node
  run_traj_control
  move_right_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############
# (如有需要可添加测试)
