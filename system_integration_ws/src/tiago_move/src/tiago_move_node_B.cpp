#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal createGoal(double x, double y, double yaw)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return goal;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigate_to_table_b");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveBaseClient ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();

    std::vector<double> waypoint_B;
    if (!nh.getParam("waypoint_table_B", waypoint_B) || waypoint_B.size() != 3) {
        ROS_ERROR("Failed to load waypoint_table_B");
        return -1;
    }

    ROS_INFO("Navigating to Table B...");
    ac.sendGoal(createGoal(waypoint_B[0], waypoint_B[1], waypoint_B[2]));
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reached Table B successfully.");
    } else {
        ROS_ERROR("Failed to reach Table B.");
        return -1;
    }

    return 0;
}
