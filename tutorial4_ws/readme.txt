Please clone the darknet_ros to src
Please compile the project

source your_tiago_path/devel/setup.bash
catkin build
source devel/setup.bash

Task 1: 
1. Please launch the world with beercans. You can see 2 beercans on the table.
roslaunch object_detection_world tiago.launch world_suffix:=beercan

2.If the robot is not looking at the table, pleale run:

rosrun controllers_tutorials run_traj_control

3. Now we run the object detection function:

roslaunch object_detection_world object_detection.launch



Task 2:
1. Now, we segment the plane:

roslaunch plane_segmentation plane_segmentation_tiago.launch

Task 3:
1. Now, we run the labeling node:

roslaunch object_labeling object_labeling.launch 

You can see the result in rviz.
