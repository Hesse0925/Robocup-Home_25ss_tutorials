#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import tf2_ros
import tf.transformations


class WallYawController:
    def __init__(self):
        rospy.init_node('wall_yaw_controller')

        self.target_yaw = None
        self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/desired_wall_yaw', Float64, self.yaw_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 控制参数
        self.Kp = rospy.get_param("~Kp", 0.8)
        self.Kd = rospy.get_param("~Kd", 0.1)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.3)

        self.last_error = 0.0
        self.control_rate = rospy.Rate(10)  # 10 Hz
        self.dt = 1.0 / 10.0

        rospy.loginfo("✅ WallYawController with PD control initialized.")
        self.control_loop()

    def yaw_callback(self, msg):
        self.target_yaw = msg.data

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.target_yaw is None:
                self.control_rate.sleep()
                continue

            try:
                trans = self.tf_buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(0.5))
                q = trans.transform.rotation
                quat = [q.x, q.y, q.z, q.w]
                _, _, current_yaw = tf.transformations.euler_from_quaternion(quat)

                # yaw error in [-pi, pi]
                yaw_error = self.target_yaw - current_yaw
                yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

                if abs(np.degrees(yaw_error)) < 1.0:
                    rospy.loginfo("✅ Yaw aligned within 1°, shutting down controller.")
                    self.stop_robot()
                    rospy.signal_shutdown("Yaw alignment complete")
                    return

                # ===== PD 控制器 =====
                error_derivative = (yaw_error - self.last_error) / self.dt
                angular_z = self.Kp * yaw_error + self.Kd * error_derivative
                self.last_error = yaw_error

                # 限制角速度
                angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)

                cmd = Twist()
                cmd.angular.z = angular_z
                self.cmd_pub.publish(cmd)

                rospy.loginfo(f"🧭 Yaw error: {np.degrees(yaw_error):.1f}°, cmd: {angular_z:.3f}")

            except Exception as e:
                rospy.logwarn("❌ TF lookup failed: " + str(e))

            self.control_rate.sleep()

    def stop_robot(self):
        cmd = Twist()
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        WallYawController()
    except rospy.ROSInterruptException:
        pass
