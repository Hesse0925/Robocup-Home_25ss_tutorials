cmake_minimum_required(VERSION 2.8.3)
project(controllers_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  realtime_tools
  controller_interface
  actionlib
  trajectory_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/new_head_controller.cpp
  # src/combined_resource_controller.cpp  # 
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#  run_traj_control 
add_executable(run_traj_control src/run_traj_control.cpp)
target_link_libraries(run_traj_control ${catkin_LIBRARIES})

add_executable(build_map_controller src/build_map_controller.cpp)
target_link_libraries(build_map_controller ${catkin_LIBRARIES})

add_executable(raise_head src/raise_head.cpp)
target_link_libraries(raise_head ${catkin_LIBRARIES})

# plugin
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# 
install(TARGETS run_traj_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 
foreach(dir config launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(FILES tutorial_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

