#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class NoCupResponder
{
public:
  NoCupResponder()
    : ac_("move_base", true), triggered_(false)
  {
    ros::NodeHandle nh;
    sub_ = nh.subscribe("/no_cup_detected", 1, &NoCupResponder::callback, this);
    ROS_INFO("Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("NoCupResponder is ready.");
  }

  void callback(const std_msgs::Empty::ConstPtr&)
  {
    if (triggered_)
      return;

    triggered_ = true;
    ROS_INFO("No cup detected. Moving right by 1 meter...");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.y = -1.0;  // right
    goal.target_pose.pose.position.x = 0.1;  // right
    goal.target_pose.pose.orientation.w = 1.1;

    ac_.sendGoal(goal);
    ac_.waitForResult();

    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully moved right.");
    else
      ROS_WARN("Movement failed.");

    ros::shutdown();  // Stop the node after the task is complete
  }

private:
  ros::Subscriber sub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  bool triggered_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "no_cup_react_node");
  NoCupResponder node;
  ros::spin();
  return 0;
}
