1. replace the world and map file path to yours in 
/src/ics_gazebo/launch/tiago_main.launch

2. open the main tiago world
->roslaunch ics_gazebo tiago_main.launch 

3. localization until it is correct
->roslaunch ics_gazebo tiago_map_localization.launch 

4. run perception_pipeline (lower head+plane_segmentation+object_detection+object_labeling)
->roslaunch ics_gazebo perception_pipeline.launch 

5. run state machine
->rosrun hsrb_task_manager task_manager.py

机器人initial pose:
position.x = 0.1475;
position.y = 0.1602;
position.z = 0.6295;

orientation.x = 0.6907;
orientation.y = 0.0230;
orientation.z = 0.7225;
orientation.w = 0.0201;

或者
roll  = 0.9360;
pitch = -1.4950;
yaw   = 2.2096;

代码为：
home_pose.pose.position.x = 0.1475;
home_pose.pose.position.y = 0.1602;
home_pose.pose.position.z = 0.6295;

tf::Quaternion q;
q.setRPY(0.9360, -1.4950, 2.2096);  // Original RPY from current pose

## voxblox 
roslaunch voxblox_ros tiago_voxblox_base.launch
在rviz中add mesh查看建模

->rosrun controllers_tutorials build_map_controller 实现头部周期运动填补建模空隙



################
登陆tiago
ssh pal@tiago-46c
密码：pal
###############

################
杀原pal_map_manager的/map进程
rostopic info /map
rosnode kill /pal_map_manager_tiago_46c_
###############

################
打开octomap的命令
roslaunch tiago_moveit_config move_group.launch camera:=true pipeline:=chomp
###############

###############
打开roslaunch ics_gazebo tiago_main_real.launch（实机）
或者roslaunch ics_gazebo tiago_main.launch （仿真）
###############
roslaunch ics_gazebo tiago_map_localization.launch
roslaunch pal_navigation_cfg_tiago move_base.launch public_sim:=true local_planner:=teb
###############
打开red_marker_detection
rosrun red_marker_detector red_marker_detector_node.py
rosrun red_marker_detector red_marker_detector_top3.py
###############

###############
开启探测，2为刷新速率
rosrun topic_tools throttle messages /xtion/depth_registered/points 1  /throttle_filtering_points/filtered_points 
rosrun topic_tools throttle messages /xtion/depth_registered/points 2  /throttle_filtering_points/filtered_points 
###############

###############
导航到桌子A
roslaunch tiago_move tiago_move_A.launch

导航到桌子B
roslaunch tiago_move tiago_move_B.launch
###############

###############
运动头部进行扫描
rosparam load `rospack find tiago_moveit_tutorial`/config/tiago_octomap_motions.yaml
rosrun play_motion run_motion head_look_around
###############

###############
base旋转正对墙壁
rosrun red_marker_detector wall_detection.py
rosrun red_marker_detector controller.py
###############

###############
头部运动让三个红色圆位于视野中心
rosrun controllers_tutorials run_traj_control
###############

###############
执行抓取
rosrun tiago_move tiago_grasp_node
rosrun tiago_move grip_with_index _marker_index:=0
###############

###############
执行放置
rosrun tiago_move tiago_grasp_node
rosrun tiago_move tiago_place_node _marker_index:=0
###############

###############
运行状态机
roslaunch tiago_task_manager run_state_machine.launch
###############