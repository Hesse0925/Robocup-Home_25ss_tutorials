cmake_minimum_required(VERSION 2.8.3)
project(ics_gazebo)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  smach
  smach_ros
  )

catkin_package()

foreach(dir config launch Media models scripts worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(PROGRAMS
  src/sequential_launcher_smach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


