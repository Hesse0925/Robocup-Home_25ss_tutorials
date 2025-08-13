#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
import tf2_geometry_msgs


class WallPoseEstimatorOpen3D:
    def __init__(self):
        rospy.init_node('wall_pose_estimator_open3d')

        # 订阅点云
        rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, self.pointcloud_callback)

        # 发布墙面 pose 和 marker
        self.pose_pub = rospy.Publisher('/wall_pose', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/wall_marker', Marker, queue_size=1)

        # 发布目标 yaw（机器人应朝向墙面法向的方向）
        self.yaw_pub = rospy.Publisher('/desired_wall_yaw', Float64, queue_size=1)

        # TF2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("✅ Open3D-based wall pose estimator initialized.")
        rospy.spin()

    def pointcloud_callback(self, cloud_msg):
        # 获取点云数组
        points = list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")))
        if len(points) < 100:
            return

        xyz = np.array(points, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        # 拟合平面（墙面）
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        [a, b, c, d] = plane_model
        normal = np.array([a, b, c], dtype=np.float64)
        normal /= np.linalg.norm(normal)

        wall_points = np.asarray(pcd.points)[inliers]
        center = np.mean(wall_points, axis=0)

        # ✅ 确保法向量朝向相机（即与从相机指向墙面的视线相反）
        view_vec = center / np.linalg.norm(center)
        if np.dot(normal, view_vec) > 0:
            normal = -normal  # 翻转，使法向量指向相机

        # 构造旋转矩阵（Z = 法向量）
        z = normal
        x = np.cross([0, 1, 0], z)
        if np.linalg.norm(x) < 1e-3:
            x = np.cross([1, 0, 0], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R = np.column_stack((x, y, z))
        quat = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((R, [[0], [0], [0]])), [0, 0, 0, 1]))
        )

        # 发布墙面 pose（在相机坐标系）
        pose = PoseStamped()
        pose.header = cloud_msg.header  # frame_id = xtion_rgb_optical_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat
        self.pose_pub.publish(pose)

        # 可视化 marker（蓝色箭头表示墙面法向）
        self.publish_marker(center, R, cloud_msg.header.frame_id)

        # ✅ 将姿态转换到 base_footprint 坐标系
        try:
            transformed_pose = self.tf_buffer.transform(
                pose, "base_footprint", rospy.Duration(1.0)
            )
            q = transformed_pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            R_tf = tf.transformations.quaternion_matrix(quat)[:3, :3]
            wall_normal = R_tf[:, 2]  # Z轴为墙面朝外

            # 控制目标：朝向墙面，即 -Z 轴
            target_dir = -wall_normal
            target_yaw = np.arctan2(target_dir[1], target_dir[0])

            yaw_msg = Float64()
            yaw_msg.data = target_yaw + np.pi + 0.4
            self.yaw_pub.publish(yaw_msg)

            rospy.loginfo(f"🎯 Target yaw (in base_footprint): {np.degrees(target_yaw):.2f}°")

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("❌ TF transform to base_footprint failed.")

    def publish_marker(self, origin, R, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wall_axes"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        from geometry_msgs.msg import Point
        start = Point(*origin)
        end = Point(*(origin + 0.3 * R[:, 2]))  # Z轴
        marker.points = [start, end]
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        WallPoseEstimatorOpen3D()
    except rospy.ROSInterruptException:
        pass
