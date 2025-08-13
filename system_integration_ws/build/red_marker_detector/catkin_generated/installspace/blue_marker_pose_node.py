#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, Quaternion



class BlueMarkerPoseEstimator:
    def __init__(self):
        rospy.init_node('blue_marker_pose_node')

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.image = None

        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/xtion/rgb/camera_info', CameraInfo, self.camera_info_callback)

        self.point_pub = rospy.Publisher('/blue_marker_pose/position', PointStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/blue_marker_pose/axes', Marker, queue_size=3)
        self.pose_pub = rospy.Publisher('/blue_marker_pose/pose', PoseStamped, queue_size=1)
        rospy.loginfo("✅ Blue marker pose node initialized.")
        rospy.spin()

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def image_callback(self, msg):
        if None in (self.fx, self.fy, self.cx, self.cy):
            return

        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_estimate_pose()

    def detect_and_estimate_pose(self):
        img = self.image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 深蓝色阈值范围（根据打印颜色调整）
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            if len(approx) == 4 and cv2.contourArea(approx) > 1000:
                corners = approx[:, 0, :]

                # 排序角点顺序为 TL, TR, BR, BL
                corners = sorted(corners, key=lambda p: p[0] + p[1])
                tl = corners[0]
                br = corners[3]
                remain = [p for p in corners if not np.array_equal(p, tl) and not np.array_equal(p, br)]
                if remain[0][0] < remain[1][0]:
                    tr, bl = remain[1], remain[0]
                else:
                    tr, bl = remain[0], remain[1]
                image_points = np.array([tl, tr, br, bl], dtype=np.float32)

                # 模型坐标 (120mm 方块)
                model_points = np.array([
                    [0.0, 0.0, 0.0],
                    [0.12, 0.0, 0.0],
                    [0.12, 0.12, 0.0],
                    [0.0, 0.12, 0.0]
                ], dtype=np.float32)

                camera_matrix = np.array([
                    [self.fx, 0, self.cx],
                    [0, self.fy, self.cy],
                    [0, 0, 1]
                ])
                dist_coeffs = np.zeros((4, 1))

                success, rvec, tvec = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)
                if not success:
                    return
                
                # 发布 6D 位姿
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "xtion_rgb_optical_frame"

                # 设置位置
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # 设置方向（rvec -> 四元数）
                R, _ = cv2.Rodrigues(rvec)
                quat = tf_trans.quaternion_from_matrix(np.vstack((np.hstack((R, [[0],[0],[0]])), [0,0,0,1])))  # 4x4 matrix
                pose_msg.pose.orientation = Quaternion(*quat)

                self.pose_pub.publish(pose_msg)

                rospy.loginfo(f"🔵 Pose: position=({tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}), "
                f"orientation=({quat[0]:.2f}, {quat[1]:.2f}, {quat[2]:.2f}, {quat[3]:.2f}) [xyzw]")


                # 发布 3D 坐标（相机坐标系下）
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "xtion_rgb_optical_frame"
                point_msg.point.x = tvec[0][0]
                point_msg.point.y = tvec[1][0]
                point_msg.point.z = tvec[2][0]
                self.point_pub.publish(point_msg)

                # rospy.loginfo(f"Blue marker pose: ({tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f})")

                # 可视化坐标轴
                self.publish_axes(tvec, rvec)
                return

    def publish_axes(self, tvec, rvec):
        R, _ = cv2.Rodrigues(rvec)
        T = tvec.reshape(3)
        axis_length = 0.05  # 5cm

        # 相机坐标系下的坐标轴终点
        axes = np.eye(3) * axis_length
        ends = R @ axes + T[:, np.newaxis]

        origin = T.tolist()
        for i, color in zip(range(3), [(1,0,0,1), (0,1,0,1), (0,0,1,1)]):
            end = ends[:, i].tolist()
            self.marker_pub.publish(self.create_arrow_marker(i, origin, end, color))

    def create_arrow_marker(self, marker_id, start, end, color):
        marker = Marker()
        marker.header.frame_id = "xtion_rgb_optical_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "blue_marker_axes"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        from geometry_msgs.msg import Point
        p_start = Point(*start)
        p_end = Point(*end)
        marker.points = [p_start, p_end]

        marker.scale.x = 0.02
        marker.scale.y = 0.04
        marker.scale.z = 0.06

        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0.2)
        return marker

if __name__ == "__main__":
    try:
        BlueMarkerPoseEstimator()
    except rospy.ROSInterruptException:
        pass
