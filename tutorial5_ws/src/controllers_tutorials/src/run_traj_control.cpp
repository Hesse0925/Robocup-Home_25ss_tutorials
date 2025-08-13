#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControlClient;
typedef boost::shared_ptr<HeadControlClient> HeadControlClientPtr;

// Connect to head controller action server
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

// Create a trajectory that moves head downward (pitch only)
void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal, double unused1, double unused2)
{
  goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
  goal.trajectory.points.resize(1);

  // Only control head_2_joint to look down (pitch), keep head_1_joint still
  goal.trajectory.points[0].positions = {0.0, -0.8};  // yaw = 0.0, pitch down ≈ 45°
  goal.trajectory.points[0].velocities = {0.0, 0.0};
  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "run_head_traj_control");
  ros::NodeHandle nh;

  ROS_INFO("Starting continuous head shaking...");

  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))
  {
    ROS_FATAL("Timed out waiting for valid time.");
    return EXIT_FAILURE;
  }

  HeadControlClientPtr HeadClient;
  createHeadClient(HeadClient);

  control_msgs::FollowJointTrajectoryGoal goal;
  waypoints_head_goal(goal, 0.0, 0.0);  // unused params
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
  HeadClient->sendGoal(goal);
  HeadClient->waitForResult(ros::Duration(4.0));

  ROS_INFO("Sent head down trajectory.");

  return EXIT_SUCCESS;
}
