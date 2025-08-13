#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter()
  {
    sub_ = nh_.subscribe("/object_centroid", 10, &KalmanFilter::callback, this);
    pub_ = nh_.advertise<geometry_msgs::PointStamped>("/filtered_centroid", 1);

    x_ = Eigen::Vector3f::Zero();     // Initial state
    P_ = Eigen::Matrix3f::Identity(); // Initial uncertainty

    Q_ = 0.0001 * Eigen::Matrix3f::Identity(); // Process noise
    R_ = 0.0005 * Eigen::Matrix3f::Identity(); // Measurement noise
    initialized_ = false;
  }

  void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    Eigen::Vector3f z;
    z << msg->point.x, msg->point.y, msg->point.z;

    if (!initialized_)
    {
      x_ = z;
      initialized_ = true;
      ROS_INFO("Kalman filter initialized.");
    }
    else
    {
      // Prediction step (static model, so x stays the same)
      Eigen::Vector3f x_pred = x_;
      Eigen::Matrix3f P_pred = P_ + Q_;

      // Update step
      Eigen::Matrix3f K = P_pred * (P_pred + R_).inverse();
      x_ = x_pred + K * (z - x_pred);
      P_ = (Eigen::Matrix3f::Identity() - K) * P_pred;
    }

    // Publish filtered position
    geometry_msgs::PointStamped filtered_msg = *msg;
    filtered_msg.point.x = x_(0);
    filtered_msg.point.y = x_(1);
    filtered_msg.point.z = x_(2);
    pub_.publish(filtered_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  Eigen::Vector3f x_;
  Eigen::Matrix3f P_;
  Eigen::Matrix3f Q_;
  Eigen::Matrix3f R_;

  bool initialized_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_kalman_filter");
  KalmanFilter kf;
  ros::spin();
  return 0;
}
