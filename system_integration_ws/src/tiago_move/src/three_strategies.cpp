#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <Eigen/Geometry>

enum GraspStrategy {
    TOP_GRASP,
    SIDE_GRASP,
    ANGLED_GRASP
};

class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          gripper_client_("/gripper_controller/follow_joint_trajectory", true),
          play_motion_client_("play_motion", true),
          grasp_executed_(false)
    {}

    void initialize() {
        arm_group_.setPlannerId("RRTConnectkConfigDefault");
        arm_group_.setPoseReferenceFrame("base_footprint");
        arm_group_.setStartStateToCurrentState();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPlanningTime(5.0);

        ros::NodeHandle nh;
        pose_sub_ = nh.subscribe("/object_pose", 1, &GraspController::poseCallback, this);

        ROS_INFO("Waiting for gripper action server...");
        gripper_client_.waitForServer();
        ROS_INFO("Gripper action server connected.");

        ROS_INFO("Waiting for play_motion action server...");
        play_motion_client_.waitForServer();
        ROS_INFO("play_motion action server connected.");
        ros::Duration(2.0).sleep();

        ROS_INFO("GraspController initialized and waiting for object pose...");
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (grasp_executed_) return;
        

        geometry_msgs::PoseStamped object_pose = *msg;
        ROS_INFO("Received object pose at [%.2f, %.2f, %.2f]",
                 object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

        GraspStrategy strategy = chooseStrategy(object_pose.pose.orientation);

        if (strategy != SIDE_GRASP) {
            if (!moveToSafePose()) return;
        }
        const char* strategy_name = nullptr;
        switch (strategy) {
            case TOP_GRASP:    strategy_name = "TOP_GRASP (top-down grasp, object lying flat)"; break;
            case SIDE_GRASP:   strategy_name = "SIDE_GRASP (side grasp, object standing up)"; break;
            case ANGLED_GRASP: strategy_name = "ANGLED_GRASP (angled grasp, object tilted)"; break;
            default:           strategy_name = "UNKNOWN"; break;
        }

        ROS_INFO("Received object pose at [%.2f, %.2f, %.2f], orientation [x=%.2f, y=%.2f, z=%.2f, w=%.2f]",
                object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z,
                object_pose.pose.orientation.x, object_pose.pose.orientation.y,
                object_pose.pose.orientation.z, object_pose.pose.orientation.w);

        ROS_INFO("Selected grasp strategy: %s", strategy_name);

        openGripper();
        ros::Duration(0.5).sleep();

        // ✅ 尝试抓取
        if (performGrasp(object_pose, strategy)) {
            ROS_INFO("Grasp succeeded!");
            grasp_executed_ = true;  // ✅ 只在成功后设置为 true
        } else {
            ROS_WARN("Grasp failed.");
            grasp_executed_ = false; // ✅ 如果失败，保留执行权限
        }
        
        // ros::shutdown();
    }

    GraspStrategy chooseStrategy(const geometry_msgs::Quaternion& orientation) {
        Eigen::Quaternionf q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f rot = q.toRotationMatrix();
        Eigen::Vector3f z_axis = rot.col(2);

        float cos_angle = z_axis.dot(Eigen::Vector3f::UnitZ());

        if (fabs(cos_angle) > 0.85) return TOP_GRASP;
        else if (fabs(cos_angle) < 0.3) return SIDE_GRASP;
        else return ANGLED_GRASP;
    }

    bool moveToSafePose() {
        std::map<std::string, double> mid_joint_pose = {
            {"arm_1_joint", 0.11}, {"arm_2_joint", 0.20}, {"arm_3_joint", -0.20},
            {"arm_4_joint", 1.94}, {"arm_5_joint", -1.57}, {"arm_6_joint", 1.37},
            {"arm_7_joint", 0.00}, {"torso_lift_joint", 0.20}
        };

        arm_group_.setJointValueTarget(mid_joint_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (arm_group_.plan(plan) && arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return true;
        }
        return false;
    }

    bool performGrasp(const geometry_msgs::PoseStamped& pose, GraspStrategy strategy) {
        bool success = false;
    
        // ======= SIDE_GRASP 改为纯 MoveIt 控制 ============
        if (strategy == ANGLED_GRASP) {
            ROS_INFO("Using ANGLED_GRASP with MoveIt");
        
            // 1. Move to angled_grasp_pose
            std::map<std::string, double> angled_grasp_pose = {
                {"torso_lift_joint", 0.245}, {"arm_1_joint", 0.262}, {"arm_2_joint", 0.561},
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
        
            // 2. 当前姿态微调到夹取点
            geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
            geometry_msgs::PoseStamped grasp_pose = current_pose;
        
            // ✅ 向目标物体方向靠近一点（例如降低 Z + 向前 Y 方向推进）
            grasp_pose.pose.position.z += 0.10;  // 向下
            grasp_pose.pose.position.y -= 0.05;  // 向前（你可以根据相机位置微调）
        
            ROS_INFO("Approaching object from current pose to grasp...");
            if (!moveToPose(grasp_pose)) {
                ROS_WARN("Failed to reach final grasp pose.");
                return false;
            }
        
            // 3. 闭合夹爪
            ros::Duration(0.5).sleep();
            closeGripper();
            ros::Duration(0.5).sleep();
        
            success = true;
        }
    
        else if (strategy == SIDE_GRASP) {
            ROS_INFO("Using SIDE_GRASP with MoveIt");
        
            std::map<std::string, double> side_grasp_pose = {
                {"torso_lift_joint", 0.086}, {"arm_1_joint", 0.084}, {"arm_2_joint", -0.795},
                {"arm_3_joint", -0.243}, {"arm_4_joint", 2.255}, {"arm_5_joint", 0.107},
                {"arm_6_joint", 1.381}, {"arm_7_joint", -1.457}
            };
            arm_group_.setJointValueTarget(side_grasp_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_ERROR("Failed to plan to side_grasp_pose");
                return false;
            }
            arm_group_.execute(plan);
            ros::Duration(0.5).sleep();
        
            geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
            geometry_msgs::PoseStamped grasp_pose = current_pose;
            grasp_pose.pose.position.z += 0.0;
            grasp_pose.pose.position.x -= 0.10;
        
            ROS_INFO("Approaching object from current pose to grasp...");
            if (!moveToPose(grasp_pose)) {
                ROS_WARN("Failed to reach final grasp pose.");
                return false;
            }
        
            ros::Duration(0.5).sleep();
            closeGripper();
            ros::Duration(0.5).sleep();
        
            success = true;
        }

        else if (strategy == TOP_GRASP) {
            ROS_INFO("Using TOP_GRASP with MoveIt");

            std::map<std::string, double> top_grasp_pose = {
                {"torso_lift_joint", 0.192}, {"arm_1_joint", 2.232}, {"arm_2_joint", 0.959},
                {"arm_3_joint", 0.993}, {"arm_4_joint", 1.323}, {"arm_5_joint", -1.058},
                {"arm_6_joint", 1.394}, {"arm_7_joint", -0.005}
            };

            // 1. 移动到 top grasp 初始姿态
            arm_group_.setJointValueTarget(top_grasp_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (arm_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_ERROR("Failed to plan to top_grasp_pose");
                return false;
            }
            arm_group_.execute(plan);
            ros::Duration(0.5).sleep();

            // 2. 从当前位置向下靠近目标物体
            geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
            geometry_msgs::PoseStamped grasp_pose = current_pose;
            grasp_pose.pose.position.z -= 0.10;  // 垂直向下靠近

            ROS_INFO("Approaching object from current pose to grasp...");
            if (!moveToPose(grasp_pose)) {
                ROS_WARN("TOP_GRASP failed to reach final grasp pose.");
                return false;
            }

            // 3. 闭合夹爪
            ros::Duration(0.5).sleep();
            closeGripper();
            ros::Duration(0.5).sleep();

            success = true;
        }


        else {
            ROS_WARN("Unknown strategy.");
            return false;
        }
        
    
        // ======= 抓取成功后执行 post_grasp_lift ============
        if (success) {
            std::map<std::string, double> post_lift_pose = {
                {"torso_lift_joint", 0.226}, {"arm_1_joint", 0.366}, {"arm_2_joint", 0.137},
                {"arm_3_joint", -3.130}, {"arm_4_joint", 1.571}, {"arm_5_joint", 0.0},
                {"arm_6_joint", 0.0}, {"arm_7_joint", 0.0}
            };
            arm_group_.setJointValueTarget(post_lift_pose);
            moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
            if (arm_group_.plan(lift_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_WARN("Failed to lift after grasp");
                return false;
            }
            arm_group_.execute(lift_plan);
            ros::Duration(0.5).sleep();
    
            ROS_INFO("Grasp and lift completed successfully.");
            return true;
        }
    
        return false;
    }
    
    
    bool moveToPose(const geometry_msgs::PoseStamped& pose) {
        arm_group_.setStartStateToCurrentState();
        arm_group_.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        }
        return false;
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
    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> play_motion_client_;
    bool grasp_executed_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tiago_grasp_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    GraspController controller;
    controller.initialize();

    ros::waitForShutdown();
    return 0;
}
