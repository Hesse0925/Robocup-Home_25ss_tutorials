#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControlClient;
typedef boost::shared_ptr<HeadControlClient> HeadControlClientPtr;

// 创建连接到头部控制器的 action client
void createHeadClient(HeadControlClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller...");

  actionClient.reset(new HeadControlClient("/head_controller/follow_joint_trajectory"));

  int attempts = 0, max_attempts = 5;
  while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && attempts < max_attempts)
  {
    ROS_WARN("Waiting for the head_controller action server to come up");
    ++attempts;
  }

  if (attempts == max_attempts)
    throw std::runtime_error("Head controller action server not available");
}

// 构造一个头部的“摇头+点头”轨迹
void oscillating_head_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
  goal.trajectory.points.resize(4);

  // 点的顺序：左下 -> 右下 -> 右上 -> 左上
  std::vector<std::pair<double, double>> positions = {
      {-0.5, -0.8},  // yaw -0.5 rad, pitch -0.8 rad
      { 0.5, -0.8},
      { 0.5,  0.3},
      {-0.5,  0.3}
  };

  for (size_t i = 0; i < positions.size(); ++i) {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = {positions[i].first, positions[i].second};
    pt.velocities = {0.0, 0.0};
    pt.time_from_start = ros::Duration((i + 1) * 2.0);  // 每段2秒
    goal.trajectory.points[i] = pt;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_head_motion_only");
  ros::NodeHandle nh;

  ROS_INFO("Starting head oscillation motion...");

  // 等待时间同步
  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))
  {
    ROS_FATAL("Timed out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // 初始化头部控制器
  HeadControlClientPtr HeadClient;
  createHeadClient(HeadClient);

  // 构造头部轨迹目标
  control_msgs::FollowJointTrajectoryGoal goal;
  oscillating_head_goal(goal);
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

  // 发送轨迹动作
  HeadClient->sendGoal(goal);
  HeadClient->waitForResult(goal.trajectory.points.back().time_from_start + ros::Duration(1.0));
  while (ros::ok()) {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
    HeadClient->sendGoal(goal);
    HeadClient->waitForResult(goal.trajectory.points.back().time_from_start + ros::Duration(1.0));
  }
  
  ROS_INFO("Finished head movement.");
  // return EXIT_SUCCESS;
}
