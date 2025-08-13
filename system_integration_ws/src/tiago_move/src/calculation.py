#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg

rospy.init_node('fk_solver', anonymous=True)
moveit_commander.roscpp_initialize([])

group = moveit_commander.MoveGroupCommander("arm_torso")

# 设置关节角度（与你提供的一致）
joint_values = {
    'torso_lift_joint': 0.150,
    'arm_1_joint': 0.200,
    'arm_2_joint': -1.339,
    'arm_3_joint': -0.200,
    'arm_4_joint': 1.938,
    'arm_5_joint': -1.570,
    'arm_6_joint': 1.369,
    'arm_7_joint': -0.000
}

group.set_joint_value_target(joint_values)
group.go(wait=True)

pose = group.get_current_pose().pose
print("Position:", pose.position)
print("Orientation (quaternion):", pose.orientation)
