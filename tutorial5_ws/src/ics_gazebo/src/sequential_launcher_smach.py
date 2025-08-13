#!/usr/bin/env python

import rospy
import smach
import smach_ros
import subprocess
import time

def wait_for_topic(topic, timeout=30):
    start = time.time()
    while not rospy.is_shutdown():
        topics = rospy.get_published_topics()
        if any(t[0] == topic for t in topics):
            rospy.loginfo(f"[✓] Topic {topic} is available.")
            return True
        if time.time() - start > timeout:
            rospy.logwarn(f"[×] Timeout waiting for topic {topic}")
            return False
        time.sleep(0.5)

class LaunchGazebo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo("Launching Gazebo world...")
        subprocess.Popen(["roslaunch", "ics_gazebo", "ROBOT.launch"])
        wait_for_topic("/gazebo/model_states", timeout=60)
        return 'succeeded'

class LaunchDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo("Launching object detection...")
        subprocess.Popen(["roslaunch", "object_detection_world", "object_detection.launch"])
        time.sleep(2)
        return 'succeeded'

class LaunchSegmentation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo("Launching plane segmentation...")
        subprocess.Popen(["roslaunch", "plane_segmentation", "plane_segmentation_tiago.launch"])
        time.sleep(2)
        return 'succeeded'

class LaunchLabeling(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo("Launching object labeling...")
        subprocess.Popen(["roslaunch", "object_labeling", "object_labeling.launch"])
        time.sleep(2)

        
        rospy.loginfo("Launching no_cup_react_node...")
        subprocess.Popen(["rosrun", "tiago_move", "move_right_node"])
        return 'succeeded'

class LaunchArmControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo("Launching arm control...")
        subprocess.Popen(["roslaunch", "arm_control", "arm_above_moveit.launch"])
        return 'succeeded'

def main():
    rospy.init_node('sequential_smach_launcher')

    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('LAUNCH_GAZEBO', LaunchGazebo(), transitions={'succeeded':'LAUNCH_DETECTION'})
        smach.StateMachine.add('LAUNCH_DETECTION', LaunchDetection(), transitions={'succeeded':'LAUNCH_SEGMENTATION'})
        smach.StateMachine.add('LAUNCH_SEGMENTATION', LaunchSegmentation(), transitions={'succeeded':'LAUNCH_LABELING'})
        smach.StateMachine.add('LAUNCH_LABELING', LaunchLabeling(), transitions={'succeeded':'LAUNCH_ARM_CONTROL'})
        smach.StateMachine.add('LAUNCH_ARM_CONTROL', LaunchArmControl(), transitions={'succeeded':'done'})

    sm.execute()

if __name__ == '__main__':
    main()
