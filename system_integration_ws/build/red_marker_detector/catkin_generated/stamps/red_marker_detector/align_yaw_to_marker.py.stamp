from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
import rospy, numpy as np
from geometry_msgs.msg import PoseStamped


class YawAlignController:
    def __init__(self):
        rospy.init_node("yaw_align_controller")

        self.marker_yaw = None
        self.base_yaw = None
        self.prev_error = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        rospy.Subscriber("/blue_marker_pose/pose", PoseStamped, self.marker_pose_callback)
        self.cmd_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=1)

        self.control_loop()

    def marker_pose_callback(self, msg):
        quat = msg.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        _, _, self.marker_yaw = euler_from_quaternion(q)

    def get_base_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(0.5))
            q = trans.transform.rotation
            _, _, self.base_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        except:
            rospy.logwarn("TF lookup failed.")
            self.base_yaw = None

    def control_loop(self):
        rate = rospy.Rate(10)
        Kp = 1.2
        Kd = 0.05

        while not rospy.is_shutdown():
            if self.marker_yaw is None:
                rate.sleep()
                continue

            self.get_base_yaw()
            if self.base_yaw is None:
                rate.sleep()
                continue

            yaw_error = self.marker_yaw - self.base_yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

            d_error = (yaw_error - self.prev_error) * 10
            angular_z = Kp * yaw_error + Kd * d_error
            self.prev_error = yaw_error

            twist = Twist()
            twist.angular.z = np.clip(angular_z, -0.5, 0.5)
            self.cmd_pub.publish(twist)

            rospy.loginfo(f"Yaw error: {np.degrees(yaw_error):.2f} deg, cmd_z: {twist.angular.z:.2f}")

            

            rate.sleep()

if __name__ == "__main__":
    try:
        YawAlignController()
    except rospy.ROSInterruptException:
        pass
