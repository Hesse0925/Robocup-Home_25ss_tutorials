#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64

class YawAlignSimulation:
    def __init__(self):
        rospy.init_node("yaw_align_simulation")

        # 加载参数
        self.Kp = rospy.get_param("~Kp", 1.2)
        self.Kd = rospy.get_param("~Kd", 0.05)
        rospy.loginfo(f"[SIM] Kp = {self.Kp}, Kd = {self.Kd}")

        # 初始化模拟状态
        self.base_yaw = 0.0          # Tiago 当前方向
        self.marker_yaw = 0.0        # 蓝纸方向
        self.prev_error = 0.0

        # 发布话题
        self.yaw_ref_pub = rospy.Publisher("/yaw_ref", Float64, queue_size=1)
        self.yaw_base_pub = rospy.Publisher("/yaw_base", Float64, queue_size=1)
        self.yaw_error_pub = rospy.Publisher("/yaw_error", Float64, queue_size=1)
        self.cmd_angular_pub = rospy.Publisher("/cmd_angular_z", Float64, queue_size=1)

        self.run_simulation()

    def run_simulation(self):
        rate = rospy.Rate(10)  # 10Hz
        dt = 0.1
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()

            # 模拟 yaw_ref 的阶跃信号（第 5 秒从 0 跳到 45°）
            self.marker_yaw = 0.0 if t < 5.0 else np.deg2rad(45)

            # 控制器计算
            yaw_error = self.marker_yaw - self.base_yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))  # wrap to [-pi, pi]
            d_error = (yaw_error - self.prev_error) / dt
            angular_z = self.Kp * yaw_error + self.Kd * d_error
            self.prev_error = yaw_error

            # 模拟 Tiago 的 yaw 积分响应（假设角速度就是控制输入）
            self.base_yaw += angular_z * dt
            self.base_yaw = np.arctan2(np.sin(self.base_yaw), np.cos(self.base_yaw))  # wrap

            # 发布话题
            self.yaw_ref_pub.publish(Float64(self.marker_yaw))
            self.yaw_base_pub.publish(Float64(self.base_yaw))
            self.yaw_error_pub.publish(Float64(yaw_error))
            self.cmd_angular_pub.publish(Float64(angular_z))

            rospy.loginfo(f"[SIM] t={t:.1f}s, yaw_ref={np.degrees(self.marker_yaw):.1f}, "
                          f"yaw={np.degrees(self.base_yaw):.1f}, err={np.degrees(yaw_error):.1f}, "
                          f"cmd={angular_z:.2f}")

            rate.sleep()

if __name__ == "__main__":
    try:
        YawAlignSimulation()
    except rospy.ROSInterruptException:
        pass
