please clone darknet_ros in src

1. place wrs2020_cup.world to the path: /tmc_wrs_gazebo_worlds/worlds
place folder wrs2020 to the path: tiago_localization/config/wrs2020

2. open the main tiago world
->roslaunch ics_gazebo ROBOT.launch 

3. run perception_pipeline 
->roslaunch object_detection_world object_detection.launch
->roslaunch plane_segmentation plane_segmentation_tiago.launch 
->roslaunch object_labeling object_labeling.launch
->roslaunch arm_control arm_above_moveit.launch

4.You can also only run the state machine
-> rosrun ics_gazebo sequential_launcher_smach.py


5. kill all the nodes:
 pkill -f ros

