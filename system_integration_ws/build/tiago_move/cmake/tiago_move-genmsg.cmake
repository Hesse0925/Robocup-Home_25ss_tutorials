# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tiago_move: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Imoveit_msgs:/opt/ros/noetic/share/moveit_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/noetic/share/trajectory_msgs/cmake/../msg;-Ishape_msgs:/opt/ros/noetic/share/shape_msgs/cmake/../msg;-Iobject_recognition_msgs:/opt/ros/noetic/share/object_recognition_msgs/cmake/../msg;-Ioctomap_msgs:/opt/ros/noetic/share/octomap_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tiago_move_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" NAME_WE)
add_custom_target(_tiago_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tiago_move" "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" "geometry_msgs/Pose:geometry_msgs/Vector3:trajectory_msgs/JointTrajectoryPoint:geometry_msgs/PoseStamped:moveit_msgs/Grasp:trajectory_msgs/JointTrajectory:geometry_msgs/Quaternion:moveit_msgs/GripperTranslation:std_msgs/Header:geometry_msgs/Vector3Stamped:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tiago_move
  "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/Grasp.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/GripperTranslation.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago_move
)

### Generating Module File
_generate_module_cpp(tiago_move
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago_move
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tiago_move_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tiago_move_generate_messages tiago_move_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" NAME_WE)
add_dependencies(tiago_move_generate_messages_cpp _tiago_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago_move_gencpp)
add_dependencies(tiago_move_gencpp tiago_move_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago_move_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tiago_move
  "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/Grasp.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/GripperTranslation.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago_move
)

### Generating Module File
_generate_module_eus(tiago_move
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago_move
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tiago_move_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tiago_move_generate_messages tiago_move_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" NAME_WE)
add_dependencies(tiago_move_generate_messages_eus _tiago_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago_move_geneus)
add_dependencies(tiago_move_geneus tiago_move_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago_move_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tiago_move
  "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/Grasp.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/GripperTranslation.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago_move
)

### Generating Module File
_generate_module_lisp(tiago_move
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago_move
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tiago_move_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tiago_move_generate_messages tiago_move_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" NAME_WE)
add_dependencies(tiago_move_generate_messages_lisp _tiago_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago_move_genlisp)
add_dependencies(tiago_move_genlisp tiago_move_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago_move_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tiago_move
  "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/Grasp.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/GripperTranslation.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago_move
)

### Generating Module File
_generate_module_nodejs(tiago_move
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago_move
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tiago_move_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tiago_move_generate_messages tiago_move_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" NAME_WE)
add_dependencies(tiago_move_generate_messages_nodejs _tiago_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago_move_gennodejs)
add_dependencies(tiago_move_gennodejs tiago_move_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago_move_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tiago_move
  "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/Grasp.msg;/opt/ros/noetic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/moveit_msgs/cmake/../msg/GripperTranslation.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago_move
)

### Generating Module File
_generate_module_py(tiago_move
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago_move
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tiago_move_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tiago_move_generate_messages tiago_move_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laowang/workspace/system_integration_ws/src/tiago_move/srv/GenerateGrasps.srv" NAME_WE)
add_dependencies(tiago_move_generate_messages_py _tiago_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tiago_move_genpy)
add_dependencies(tiago_move_genpy tiago_move_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tiago_move_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tiago_move
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tiago_move_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET moveit_msgs_generate_messages_cpp)
  add_dependencies(tiago_move_generate_messages_cpp moveit_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tiago_move
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tiago_move_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET moveit_msgs_generate_messages_eus)
  add_dependencies(tiago_move_generate_messages_eus moveit_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tiago_move
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tiago_move_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET moveit_msgs_generate_messages_lisp)
  add_dependencies(tiago_move_generate_messages_lisp moveit_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tiago_move
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tiago_move_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET moveit_msgs_generate_messages_nodejs)
  add_dependencies(tiago_move_generate_messages_nodejs moveit_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago_move)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago_move\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tiago_move
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tiago_move_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET moveit_msgs_generate_messages_py)
  add_dependencies(tiago_move_generate_messages_py moveit_msgs_generate_messages_py)
endif()
