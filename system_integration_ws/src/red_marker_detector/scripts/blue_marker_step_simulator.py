#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import math

def publish_step_pose():
    rospy.init_node("blue_marker_step_simulator")
    pub = rospy.Publisher("/blue_marker_pose/pose", PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start_time).to_sec()
        yaw = 0 if t < 5.0 else math.radians(45)  # 5 秒后产生阶跃

        quat = quaternion_from_euler(0, 0, yaw)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 1.0
        pose.pose.orientation = Quaternion(*quat)

        pub.publish(pose)
        rate.sleep()

if __name__ == "__main__":
    publish_step_pose()
