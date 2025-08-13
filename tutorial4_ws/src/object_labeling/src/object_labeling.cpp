#include <object_labeling/object_labeling.h>

ObjectLabeling::ObjectLabeling(
    const std::string& objects_cloud_topic_,
    const std::string& camera_info_topic,
    const std::string& camera_frame) :
  is_cloud_updated_(false),
  has_camera_info_(false),
  objects_cloud_topic_(objects_cloud_topic_),
  camera_info_topic_(camera_info_topic),
  camera_frame_(camera_frame),
  K_(Eigen::Matrix3d::Zero())
{
}

ObjectLabeling::~ObjectLabeling()
{
}

bool ObjectLabeling::initalize(ros::NodeHandle& nh)
{
  // Subscribe to topics
  object_point_cloud_sub_ = nh.subscribe(objects_cloud_topic_, 10, &ObjectLabeling::cloudCallback, this);
  object_detections_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 10, &ObjectLabeling::detectionCallback, this);
  camera_info_sub_ = nh.subscribe(camera_info_topic_, 10, &ObjectLabeling::cameraInfoCallback, this);

  // Publishers
  labeled_object_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/labeled_object_point_cloud", 1);
  text_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/text_markers", 1);
  centroid_pub_ = nh.advertise<geometry_msgs::PointStamped>("cluster_centroid", 1);  // Debug only

  // Internal point clouds
  object_point_cloud_.reset(new PointCloud);
  labeled_point_cloud_.reset(new PointCloudl);

  // YOLO class dictionary
  dict_["sports ball"] = 1;
  dict_["banana"] = 2;
  dict_["cup"] = 3;
  dict_["apple"] = 4;
  dict_["bottle"] = 5;
  dict_["remote"] = 6;
  dict_["cell phone"] = 7;

  return true;
}

void ObjectLabeling::update(const ros::Time& time)
{
  if (is_cloud_updated_ && has_camera_info_)
  {
    is_cloud_updated_ = false;

    if (!labelObjects(object_point_cloud_, labeled_point_cloud_))
      return;

    // Publish labeled point cloud
    sensor_msgs::PointCloud2 labeled_cloud_msg;
    pcl::toROSMsg(*labeled_point_cloud_, labeled_cloud_msg);
    labeled_cloud_msg.header.frame_id = object_point_cloud_->header.frame_id;
    //labeled_cloud_msg.header.frame_id = camera_frame_;
    labeled_cloud_msg.header.stamp = time;
    labeled_object_cloud_pub_.publish(labeled_cloud_msg);

    // Publish visualization markers
    text_markers_.markers.clear();
    text_marker_pub_.publish(text_markers_);
  }
}

bool ObjectLabeling::labelObjects(CloudPtr& input, CloudPtrl& output)
{
  // Cluster the cloud
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(input);
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  ec.extract(cluster_indices);

  // Compute centroids
  std::vector<Eigen::Vector3d> centroids;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (const auto& idx : cluster.indices)
      cloud_cluster->push_back(input->points[idx]);

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    centroids.push_back(centroid.head<3>());

    // Publish for debug
    geometry_msgs::PointStamped msg;
    msg.header.frame_id = input->header.frame_id;
    msg.header.stamp = ros::Time::now();
    msg.point.x = centroid[0];
    msg.point.y = centroid[1];
    msg.point.z = centroid[2];
    centroid_pub_.publish(msg);
  }

  // Transform to camera frame
  tf::StampedTransform transform;
  try {
    tfListener_.lookupTransform(camera_frame_, input->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN("TF lookup failed: %s", ex.what());
    return false;
  }
  Eigen::Affine3d T;
  tf::transformTFToEigen(transform, T);
  std::vector<Eigen::Vector3d> centroids_camera;
  for (const auto& c : centroids)
    centroids_camera.push_back(T * c);

  // Project to image plane
  std::vector<Eigen::Vector2d> pixel_centroids;
  for (const auto& c : centroids_camera)
  {
    Eigen::Vector3d uvw = K_ * c;
    if (uvw.z() != 0)
      pixel_centroids.emplace_back(uvw.x() / uvw.z(), uvw.y() / uvw.z());
    else
      pixel_centroids.emplace_back(-1.0, -1.0);
  }

  // Match with YOLO detections
  std::vector<int> assigned_labels(cluster_indices.size(), 0);
  std::vector<std::string> assigned_classes(cluster_indices.size(), "unknown");
  for (const auto& bbox : detections_)
  {
    int match = findMatch(bbox, pixel_centroids);
    if (match >= 0 && dict_.find(bbox.Class) != dict_.end())
    {
      assigned_labels[match] = dict_[bbox.Class];
      assigned_classes[match] = bbox.Class;
    }
  }

  // Label the output point cloud
  output->points.clear();
  output->header = input->header;
  PointTl pt;
  size_t i = 0;
  for (const auto& cluster : cluster_indices)
  {
    for (int idx : cluster.indices)
    {
      const PointT& cpt = input->points[idx];
      pt.x = cpt.x;
      pt.y = cpt.y;
      pt.z = cpt.z;
      pt.label = assigned_labels[i];
      output->points.push_back(pt);
    }
    ++i;
  }

  // Create text markers
  text_markers_.markers.resize(assigned_classes.size());
  for (size_t i = 0; i < assigned_classes.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = assigned_classes[i];
    marker.pose.position.x = centroids[i].x();
    marker.pose.position.y = centroids[i].y();
    marker.pose.position.z = centroids[i].z() + 0.1;
    marker.color.a = 1.0;
    marker.scale.z = 0.1;
    marker.id = i;
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp = ros::Time::now();
    text_markers_.markers[i] = marker;
  }

  return true;
}

int ObjectLabeling::findMatch(const darknet_ros_msgs::BoundingBox& rect, const std::vector<Eigen::Vector2d>& centroids)
{
  double min_dist = std::numeric_limits<double>::max();
  int match_idx = -1;

  Eigen::Vector2d box_center(
    0.5 * (rect.xmin + rect.xmax),
    0.5 * (rect.ymin + rect.ymax)
  );

  for (size_t i = 0; i < centroids.size(); ++i)
  {
    double dist = (box_center - centroids[i]).norm();
    if (dist < min_dist)
    {
      min_dist = dist;
      match_idx = static_cast<int>(i);
    }
  }

  return match_idx;
}

void ObjectLabeling::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  is_cloud_updated_ = true;
  pcl::fromROSMsg(*msg, *object_point_cloud_);
  ROS_INFO_STREAM("Received object point cloud with " << object_point_cloud_->points.size() << " points.");
}

void ObjectLabeling::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
  detections_ = msg->bounding_boxes;
  ROS_INFO_STREAM("Received " << detections_.size() << " YOLO detections.");
}

void ObjectLabeling::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  has_camera_info_ = true;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      K_(i, j) = msg->K[j + i*3];
  ROS_INFO_STREAM("Camera matrix K loaded:\n" << K_);
}
