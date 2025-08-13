Task 1:
1. cd ~/your_path/tutorial2_ws
2. before you compile the tutorial2_ws, please source your tiago_ws first.

   source ~/your_taigo_package_path/devel/setup.bash
   
3. compile the packages

   catkin build
   
4. source devel/setup.bash

5. open the world with robot, door and other objects:

   roslaunch ics_gazebo ROBOT.launch world_suffix:=tutorial2
   
   open the keboard control node:
   
   rosrun key_teleop key_teleop.py
   
Task 2:
 Please use the same command as in task 1, then you can see the rviz with specific topics
 
 roslaunch ics_gazebo ROBOT.launch world_suffix:=tutorial2
 
Task 3:
1. First, publish the command in task 1 and 2:

   roslaunch ics_gazebo ROBOT.launch world_suffix:=tutorial2

2. please publish the command below and you can see the head shaking:

rosrun controllers_tutorials run_traj_control


