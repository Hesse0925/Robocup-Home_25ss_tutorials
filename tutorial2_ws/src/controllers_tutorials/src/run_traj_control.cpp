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

// Create a trajectory that moves head_1_joint from left to right (or vice versa)
void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal, double left, double right)
{
  goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
  goal.trajectory.points.resize(2);

  // Point 1
  goal.trajectory.points[0].positions = {left, 0.0};  // head_1 moves, head_2 fixed
  goal.trajectory.points[0].velocities = {0.03, 0.0};
  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  // Point 2
  goal.trajectory.points[1].positions = {right, 0.0};
  goal.trajectory.points[1].velocities = {0.03, 0.0};
  goal.trajectory.points[1].time_from_start = ros::Duration(4.0);
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

  bool toggle = false;
  const double left = -0.3;
  const double right = 0.3;

  while (ros::ok())
  {
    control_msgs::FollowJointTrajectoryGoal goal;

    if (toggle)
      waypoints_head_goal(goal, left, right);
    else
      waypoints_head_goal(goal, right, left);

    toggle = !toggle;

    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
    HeadClient->sendGoal(goal);

    ROS_INFO("Sent new head trajectory: %.1f → %.1f", goal.trajectory.points[0].positions[0],
                                                      goal.trajectory.points[1].positions[0]);

    HeadClient->waitForResult(ros::Duration(6.0));  // block until done
  }

  return EXIT_SUCCESS;
}
