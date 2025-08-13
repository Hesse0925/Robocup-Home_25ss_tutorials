#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <Eigen/Geometry>

class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          gripper_client_("/gripper_controller/follow_joint_trajectory", true),
          grasp_executed_(false)
    {}

    void initialize() {
        arm_group_.setPlannerId("chomp");
        arm_group_.setPoseReferenceFrame("base_footprint");
        arm_group_.setStartStateToCurrentState();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPlanningTime(5.0);

        ros::NodeHandle nh;
        pose_sub_ = nh.subscribe("/object_pose", 1, &GraspController::poseCallback, this);

        ROS_INFO("Waiting for gripper action server...");
        gripper_client_.waitForServer();
        ROS_INFO("Gripper action server connected.");
        ros::Duration(1.0).sleep();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (grasp_executed_) return;

        geometry_msgs::PoseStamped object_pose = *msg;
        ROS_INFO("Received object pose at [%.2f, %.2f, %.2f]",
                 object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

        openGripper();
        ros::Duration(0.5).sleep();

        if (performAngledGrasp(object_pose)) {
            ROS_INFO("Grasp succeeded!");
            grasp_executed_ = true;
        } else {
            ROS_WARN("Grasp failed.");
            grasp_executed_ = false;
        }
    }

    bool performAngledGrasp(const geometry_msgs::PoseStamped& object_pose) {
        // ✅ 不再做任何偏移补偿，直接使用 object_pose
        geometry_msgs::PoseStamped target_pose = object_pose;
    
        // ====== Step 1: 移动到初始斜角姿态 ======
        std::map<std::string, double> angled_grasp_pose = {
            {"torso_lift_joint", 0.20}, {"arm_1_joint", 0.262}, {"arm_2_joint", 0.561},
            {"arm_3_joint", -0.910}, {"arm_4_joint", 1.707}, {"arm_5_joint", -1.382},
            {"arm_6_joint", 1.334}, {"arm_7_joint", 0.932}
        };
        arm_group_.setJointValueTarget(angled_grasp_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Failed to plan to angled_grasp_pose");
            return false;
        }
        arm_group_.execute(plan);
        ros::Duration(0.5).sleep();
    
        // ====== Step 2: 从当前位置靠近目标物体 ======
        geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
        geometry_msgs::PoseStamped grasp_pose = current_pose;
        grasp_pose.pose.position.z += 0.10;  // 向下靠近
        grasp_pose.pose.position.y -= 0.05;  // 向前推进
    
        if (!moveToPose(grasp_pose)) {
            ROS_WARN("Failed to reach final grasp pose.");
            return false;
        }
    
        ros::Duration(0.5).sleep();
        closeGripper();
        ros::Duration(0.5).sleep();
    
        // ====== Step 3: 抓取后抬起 ======
        std::map<std::string, double> post_lift_pose = {
            {"torso_lift_joint", 0.226}, {"arm_1_joint", 0.366}, {"arm_2_joint", 0.137},
            {"arm_3_joint", -3.130}, {"arm_4_joint", 1.571}, {"arm_5_joint", 0.0},
            {"arm_6_joint", 0.0}, {"arm_7_joint", 0.0}
        };
        arm_group_.setJointValueTarget(post_lift_pose);
        moveit::planning_interface::MoveGroupInterface::MoveGroupInterface::Plan lift_plan;
        if (arm_group_.plan(lift_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_WARN("Failed to lift after grasp");
            return false;
        }
        arm_group_.execute(lift_plan);
        ros::Duration(0.5).sleep();
    
        return true;
    }
    
    bool moveToPose(const geometry_msgs::PoseStamped& pose) {
        arm_group_.setStartStateToCurrentState();
        arm_group_.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Planning failed.");
            return false;
        }
        
    }

    void openGripper() {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.15, 0.15};
        point.time_from_start = ros::Duration(1.0);
        goal.trajectory.points.push_back(point);
        goal.trajectory.header.stamp = ros::Time::now();
        gripper_client_.sendGoal(goal);
        gripper_client_.waitForResult();
    }

    void closeGripper() {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0};
        point.time_from_start = ros::Duration(1.0);
        goal.trajectory.points.push_back(point);
        goal.trajectory.header.stamp = ros::Time::now();
        gripper_client_.sendGoal(goal);
        gripper_client_.waitForResult();
    }

private:
    moveit::planning_interface::MoveGroupInterface arm_group_;
    ros::Subscriber pose_sub_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_client_;
    bool grasp_executed_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tiago_grasp_node_angled_only");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    GraspController controller;
    controller.initialize();

    ros::waitForShutdown();
    return 0;
}
