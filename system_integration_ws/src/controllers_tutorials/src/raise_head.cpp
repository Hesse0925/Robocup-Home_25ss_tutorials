#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <boost/shared_ptr.hpp>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControlClient;
typedef boost::shared_ptr<HeadControlClient> HeadControlClientPtr;

void createHeadClient(HeadControlClientPtr& client) {
    ROS_INFO("Connecting to head controller...");
    client.reset(new HeadControlClient("/head_controller/follow_joint_trajectory"));

    int attempts = 0, max_attempts = 5;
    while (!client->waitForServer(ros::Duration(0.5)) && ros::ok() && attempts < max_attempts) {
        ROS_WARN("Waiting for head_controller action server...");
        ++attempts;
    }

    if (attempts == max_attempts) {
        throw std::runtime_error("Head controller action server not available.");
    }
}

void sendHeadGoal(HeadControlClientPtr& client, double yaw, double pitch) {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {yaw, pitch};
    point.velocities = {0.5, 0.5};
    point.time_from_start = ros::Duration(0.5);

    goal.trajectory.points.push_back(point);
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(2.0));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "return_head_home");
    ros::NodeHandle nh;

    HeadControlClientPtr headClient;
    createHeadClient(headClient);

    ROS_INFO("Sending head to home position (yaw=0.0, pitch=0.0)");
    sendHeadGoal(headClient, 0.0, 0.0);

    ROS_INFO("Head returned to original position.");
    return 0;
}
