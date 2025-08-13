#include "tiago_move/gripper_control.h"
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <vector>

using namespace geometry_msgs;
using namespace trajectory_msgs;
using namespace moveit::planning_interface;

// 在GraspController类中添加GripperControl对象
class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          grasp_executed_(false),
          marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("grasp_markers", 1)),
          pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("grasp_pose", 1)),
          gripper_control_(nh_)  // 初始化GripperControl对象
    {}

    
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher pose_pub_;
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    bool grasp_executed_;
    GripperControl gripper_control_;  // GripperControl对象
};

GripperControl::GripperControl(ros::NodeHandle& nh) {
    gripper_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
}

void GripperControl::openGripper() {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.push_back("gripper_finger_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.04);  // 打开抓取器的目标位置
    point.time_from_start = ros::Duration(1.0);
    msg.points.push_back(point);

    gripper_pub_.publish(msg);
    ROS_INFO("Gripper opened.");
}

void GripperControl::closeGripper() {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.push_back("gripper_finger_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.0);  // 关闭抓取器的目标位置
    point.time_from_start = ros::Duration(1.0);
    msg.points.push_back(point);

    gripper_pub_.publish(msg);
    ROS_INFO("Gripper closed.");
}