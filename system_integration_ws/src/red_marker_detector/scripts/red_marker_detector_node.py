#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
class RedMarkerDetector:
    def __init__(self):
        rospy.init_node('red_marker_detector_node')
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_rgb = None
        self.latest_depth = None
        # 最多处理 N 个 marker，每个 marker 有 3 个箭头
        self.max_marker_count = 10
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/xtion/depth_registered/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/xtion/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.marker_pub = rospy.Publisher('/marker_axes', Marker, queue_size=50)
        self.point_pubs = [
            rospy.Publisher(f'/marker_3d_position_{i}', PointStamped, queue_size=1)
            for i in range(self.max_marker_count)
        ]
        rospy.loginfo("Red marker detector with sorted markers initialized.")
        rospy.spin()
    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
    def image_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_detect_markers()
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.try_detect_markers()
    def try_detect_markers(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if None in (self.fx, self.fy, self.cx, self.cy):
            return
        self.clear_old_markers()
        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 50]
        # Step 1: 提取所有 marker 位置
        marker_list = []
        for c in contours:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])
            z = float(depth[v, u])
            if z == 0 or np.isnan(z):
                continue
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            marker_list.append((x, y, z))
        # Step 2: 按 x 从小到大排序
        marker_list.sort(key=lambda pt: pt[0])
        # Step 3: 逐个编号并发布
        for i, (x, y, z) in enumerate(marker_list):
            if i >= self.max_marker_count:
                break  # 超出处理上限
            # 发布 3D 点
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "xtion_rgb_optical_frame"
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z
            self.point_pubs[i].publish(point_msg)
            rospy.loginfo(f"Sorted Marker #{i+1} at x={x:.3f}, y={y:.3f}, z={z:.3f}")
            # 发布 XYZ 坐标轴
            self.publish_axes(x, y, z, base_id=i * 10)
    def clear_old_markers(self):
        for marker_id in range(self.max_marker_count * 3):
            marker = Marker()
            marker.header.frame_id = "xtion_rgb_optical_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "marker_axes"
            marker.id = marker_id
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
    def publish_axes(self, x, y, z, base_id):
        frame_id = "xtion_rgb_optical_frame"
        timestamp = rospy.Time.now()
        length = 0.1
        self.marker_pub.publish(self.create_arrow_marker(
            marker_id=base_id + 0,
            start=(x, y, z),
            end=(x + length, y, z),
            color=(1.0, 0.0, 0.0, 1.0),  # X - 红
            frame_id=frame_id,
            timestamp=timestamp
        ))
        self.marker_pub.publish(self.create_arrow_marker(
            marker_id=base_id + 1,
            start=(x, y, z),
            end=(x, y + length, z),
            color=(0.0, 1.0, 0.0, 1.0),  # Y - 绿
            frame_id=frame_id,
            timestamp=timestamp
        ))
        self.marker_pub.publish(self.create_arrow_marker(
            marker_id=base_id + 2,
            start=(x, y, z),
            end=(x, y, z + length),
            color=(0.0, 0.0, 1.0, 1.0),  # Z - 蓝
            frame_id=frame_id,
            timestamp=timestamp
        ))
    def create_arrow_marker(self, marker_id, start, end, color, frame_id, timestamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = "marker_axes"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [Point(*start), Point(*end)]
        marker.scale.x = 0.02
        marker.scale.y = 0.04
        marker.scale.z = 0.06
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0)  # 永久可见
        return marker
if __name__ == '__main__':
    try:
        RedMarkerDetector()
    except rospy.ROSInterruptException:
        pass












