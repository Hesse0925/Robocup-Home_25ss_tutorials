#!/usr/bin/env python
import rospy
import smach
import smach_ros
import subprocess
import time

def map_grasp_to_place(grasp_index):
    return {2: 2, 1: 0, 0: 1}.get(grasp_index, -1)

class RunCommand(smach.State):
    def __init__(self, name, cmd, wait_time=0.5):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.cmd = cmd
        self.wait_time = wait_time
        self.name = name

    def execute(self, userdata):
        rospy.loginfo(f"[{self.name}] Executing: {self.cmd}")
        try:
            subprocess.Popen(self.cmd, shell=True)
            time.sleep(self.wait_time)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(f"[{self.name}] Failed: {e}")
            return 'failed'

class RunCommandBlocking(smach.State):
    def __init__(self, name, cmd):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.cmd = cmd
        self.name = name

    def execute(self, userdata):
        rospy.loginfo(f"[{self.name}] Running (blocking): {self.cmd}")
        try:
            result = subprocess.call(self.cmd, shell=True)
            return 'succeeded' if result == 0 else 'failed'
        except Exception as e:
            rospy.logerr(f"[{self.name}] Failed: {e}")
            return 'failed'

class GraspOrPlace(smach.State):
    def __init__(self, mode, index):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.mode = mode  # "grasp" or "place"
        self.index = index

    def execute(self, userdata):
        if self.mode == "grasp":
            cmd = f"rosrun tiago_move grip_with_index _marker_index:={self.index}"
        elif self.mode == "place":
            cmd = f"rosrun tiago_move tiago_place_node _marker_index:={self.index}"
        else:
            rospy.logerr("Invalid mode for GraspOrPlace")
            return 'failed'

        rospy.loginfo(f"[{self.mode.upper()}] Executing: {cmd}")
        try:
            result = subprocess.call(cmd, shell=True)
            if result != 0 and self.mode == "grasp":
                rospy.logwarn(f"[{self.mode.upper()}] Grasp failed, skipping to next round.")
            return 'succeeded' if result == 0 else 'failed'
        except Exception as e:
            rospy.logerr(f"[{self.mode.upper()}] Failed: {e}")
            return 'failed'

def create_grasp_sequence(grasp_index):
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add('OPEN_OCTOMAP',
            RunCommand("OPEN_OCTOMAP", "roslaunch tiago_moveit_config move_group.launch camera:=true", 5),
            transitions={'succeeded': 'START_FILTERING'})

        smach.StateMachine.add('START_FILTERING',
            RunCommand("START_FILTERING", "rosrun topic_tools throttle messages /xtion/depth_registered/points 1 /throttle_filtering_points/filtered_points", 2),
            transitions={'succeeded': 'HEAD_SCAN'})

        smach.StateMachine.add('HEAD_SCAN',
            RunCommand("HEAD_SCAN", "rosparam load `rospack find tiago_moveit_tutorial`/config/tiago_octomap_motions.yaml && rosrun play_motion run_motion head_look_around", 5),
            transitions={'succeeded': 'HEAD_SWEEP'})

        smach.StateMachine.add('HEAD_SWEEP',
            RunCommand("HEAD_SWEEP", "rosrun controllers_tutorials run_traj_control", 3),
            transitions={'succeeded': 'DETECT_RED'})

        smach.StateMachine.add('DETECT_RED',
            RunCommand("DETECT_RED", "rosrun red_marker_detector red_marker_detector_top3.py", 3),
            transitions={'succeeded': 'GRASP'})

        smach.StateMachine.add('GRASP',
            GraspOrPlace("grasp", grasp_index),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed'  # 显式添加失败路径
            })
    return sm

def create_place_sequence(place_index):
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add('OPEN_OCTOMAP',
            RunCommand("OPEN_OCTOMAP", "roslaunch tiago_moveit_config move_group.launch camera:=true", 5),
            transitions={'succeeded': 'START_FILTERING'})

        smach.StateMachine.add('START_FILTERING',
            RunCommand("START_FILTERING", "rosrun topic_tools throttle messages /xtion/depth_registered/points 1 /throttle_filtering_points/filtered_points", 2),
            transitions={'succeeded': 'HEAD_SCAN'})

        smach.StateMachine.add('HEAD_SCAN',
            RunCommand("HEAD_SCAN", "rosparam load `rospack find tiago_moveit_tutorial`/config/tiago_octomap_motions.yaml && rosrun play_motion run_motion head_look_around", 5),
            transitions={'succeeded': 'HEAD_SWEEP'})

        smach.StateMachine.add('HEAD_SWEEP',
            RunCommand("HEAD_SWEEP", "rosrun controllers_tutorials run_traj_control", 3),
            transitions={'succeeded': 'DETECT_RED'})

        smach.StateMachine.add('DETECT_RED',
            RunCommand("DETECT_RED", "rosrun red_marker_detector red_marker_detector_top3.py", 3),
            transitions={'succeeded': 'PLACE'})

        smach.StateMachine.add('PLACE',
            GraspOrPlace("place", place_index),
            transitions={'succeeded': 'succeeded'})
    return sm

def main():
    rospy.init_node('tiago_smach_main')

    sm_top = smach.StateMachine(outcomes=['DONE'])
    with sm_top:
        for round_index in range(3):
            place_index = map_grasp_to_place(grasp_index=round_index)
            next_round = f'RAISE_HEAD_BEFORE_A_{round_index+1}' if round_index < 2 else 'DONE'

            smach.StateMachine.add(f'RAISE_HEAD_BEFORE_A_{round_index}',
                RunCommand("RAISE_HEAD", "rosrun controllers_tutorials raise_head", 2),
                transitions={
                    'succeeded': f'NAVIGATE_TO_A_{round_index}',
                    'failed': next_round
                })

            smach.StateMachine.add(f'NAVIGATE_TO_A_{round_index}',
                RunCommand("NAV_TO_A", "roslaunch tiago_move tiago_move_A.launch", 10),
                transitions={
                    'succeeded': f'GRASP_SEQUENCE_{round_index}',
                    'failed': next_round
                })

            smach.StateMachine.add(f'GRASP_SEQUENCE_{round_index}',
                create_grasp_sequence(round_index),
                transitions={
                    'succeeded': f'RAISE_HEAD_BEFORE_B_{round_index}',
                    'failed': next_round
                })

            smach.StateMachine.add(f'RAISE_HEAD_BEFORE_B_{round_index}',
                RunCommand("RAISE_HEAD", "rosrun controllers_tutorials raise_head", 2),
                transitions={
                    'succeeded': f'NAVIGATE_TO_B_{round_index}',
                    'failed': next_round
                })

            smach.StateMachine.add(f'NAVIGATE_TO_B_{round_index}',
                RunCommand("NAV_TO_B", "roslaunch tiago_move tiago_move_B.launch", 10),
                transitions={
                    'succeeded': f'PLACE_SEQUENCE_{round_index}',
                    'failed': next_round
                })

            smach.StateMachine.add(f'PLACE_SEQUENCE_{round_index}',
                create_place_sequence(place_index),
                transitions={
                    'succeeded': next_round,
                    'failed': next_round
                })

    sis = smach_ros.IntrospectionServer('tiago_smach_server', sm_top, '/TIAGO_SMACH')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
