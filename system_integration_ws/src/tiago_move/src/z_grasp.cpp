
// ✅ 这是你的完整 C++ 抓取控制器，已修改为基于主轴方向自动生成抓取姿态

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <Eigen/Geometry>

class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          gripper_client_("/gripper_controller/follow_joint_trajectory", true),
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
        ros::Duration(2.0).sleep();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (grasp_executed_) return;
    
        geometry_msgs::PoseStamped object_pose = *msg;
    
        ROS_INFO("Received object pose at [%.2f, %.2f, %.2f]",
                 object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);
    
        // ✅ 抓取前先抬起避障
        if (!moveToSafePose()) {
            ROS_WARN("Failed to move to safe pose. Aborting grasp.");
            return;
        }
    
        // ✅ 打开夹爪准备抓取
        openGripper();
        ros::Duration(0.5).sleep();
    
        // ✅ 尝试抓取
        if (performAdaptiveGrasp(object_pose)) {
            ROS_INFO("Grasp succeeded!");
            grasp_executed_ = true;
    
            // ✅ 抓取成功后抬起
            std::map<std::string, double> post_lift_pose = {
                {"torso_lift_joint", 0.226}, {"arm_1_joint", 0.366}, {"arm_2_joint", 0.137},
                {"arm_3_joint", -3.130}, {"arm_4_joint", 1.571}, {"arm_5_joint", 0.0},
                {"arm_6_joint", 0.0}, {"arm_7_joint", 0.0}
            };
            arm_group_.setJointValueTarget(post_lift_pose);
            moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
            if (arm_group_.plan(lift_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_WARN("Failed to lift after grasp");
                return;
            }
            arm_group_.execute(lift_plan);
            ros::Duration(0.5).sleep();
            ROS_INFO("Lift after grasp completed.");
        } else {
            ROS_WARN("Grasp failed.");
        }
    }
    

    bool performAdaptiveGrasp(const geometry_msgs::PoseStamped& object_pose) {
        // ✅ 第一步：根据 gripper Z 轴方向对 object_pose 做位置补偿（后退 20cm）
        geometry_msgs::PoseStamped adjusted_pose = object_pose;
    
        Eigen::Quaternionf q(object_pose.pose.orientation.w,
                             object_pose.pose.orientation.x,
                             object_pose.pose.orientation.y,
                             object_pose.pose.orientation.z);
        Eigen::Matrix3f R = q.toRotationMatrix();
        Eigen::Vector3f gripper_z = -R.col(2).normalized();  // 抓取方向朝向物体
    
        Eigen::Vector3f obj_pos(object_pose.pose.position.x,
                                object_pose.pose.position.y,
                                object_pose.pose.position.z);
        Eigen::Vector3f offset = +0.20f * gripper_z;  // ✅ 向后退回 20cm，使 arm_7_link 成为控制目标
        obj_pos += offset;
    
        adjusted_pose.pose.position.x = obj_pos.x();
        adjusted_pose.pose.position.y = obj_pos.y();
        adjusted_pose.pose.position.z = obj_pos.z();
    
        // ✅ 第二步：使用补偿后的位姿来生成 grasp pose
        geometry_msgs::PoseStamped grasp_pose;
        if (!generateGraspPose(adjusted_pose, grasp_pose)) {
            ROS_ERROR("Failed to generate adaptive grasp pose.");
            return false;
        }
    
        // ✅ 第三步：生成 pregrasp pose（在 grasp pose 基础上后退 20cm）
        geometry_msgs::PoseStamped pregrasp_pose = grasp_pose;
        Eigen::Quaternionf q_grasp(grasp_pose.pose.orientation.w,
                                   grasp_pose.pose.orientation.x,
                                   grasp_pose.pose.orientation.y,
                                   grasp_pose.pose.orientation.z);
        Eigen::Matrix3f R_grasp = q_grasp.toRotationMatrix();
        Eigen::Vector3f approach_dir = -R_grasp.col(2).normalized();  // negative Z: 退回路径
    
        pregrasp_pose.pose.position.x += 0.20 * approach_dir(0);
        pregrasp_pose.pose.position.y += 0.20 * approach_dir(1);
        pregrasp_pose.pose.position.z += 0.20 * approach_dir(2);
    
        // ✅ 执行 pregrasp → grasp → close gripper
        if (!moveToPose(pregrasp_pose)) return false;
        ros::Duration(0.5).sleep();
    
        if (!moveToPose(grasp_pose)) return false;
        ros::Duration(0.5).sleep();
    
        closeGripper();
        ros::Duration(0.5).sleep();
    
        return true;
    }
    

    bool moveToSafePose() {
        std::map<std::string, double> mid_joint_pose = {
            {"arm_1_joint", 0.11}, {"arm_2_joint", 0.20}, {"arm_3_joint", -0.20},
            {"arm_4_joint", 1.94}, {"arm_5_joint", -1.57}, {"arm_6_joint", 1.37},
            {"arm_7_joint", 0.00}, {"torso_lift_joint", 0.10}
        };
    
        arm_group_.setJointValueTarget(mid_joint_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
    
        if (arm_group_.plan(plan) &&
            arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            return true;
        }
        ROS_WARN("Failed to move to safe pose.");
        return false;
    }
    
    bool generateGraspPose(const geometry_msgs::PoseStamped& obj_pose, geometry_msgs::PoseStamped& grasp_pose_out) {
        Eigen::Quaternionf q(obj_pose.pose.orientation.w,
                             obj_pose.pose.orientation.x,
                             obj_pose.pose.orientation.y,
                             obj_pose.pose.orientation.z);
        Eigen::Matrix3f R_obj = q.toRotationMatrix();
        Eigen::Vector3f obj_z = R_obj.col(2);

        Eigen::Vector3f gripper_z = -obj_z.normalized();
        Eigen::Vector3f up(0, 0, 1);
        if (fabs(gripper_z.dot(up)) > 0.95) up = Eigen::Vector3f(1, 0, 0);

        Eigen::Vector3f gripper_x = up.cross(gripper_z).normalized();
        Eigen::Vector3f gripper_y = gripper_z.cross(gripper_x);

        Eigen::Matrix3f R_grasp;
        R_grasp.col(0) = gripper_x;
        R_grasp.col(1) = gripper_y;
        R_grasp.col(2) = gripper_z;

        Eigen::Quaternionf q_grasp(R_grasp);

        grasp_pose_out = obj_pose;
        grasp_pose_out.pose.position.z += 0.05;  // ✅ 向上偏移 5 cm，防止夹爪撞桌面
        // grasp_pose_out.pose.position.y -= 0.05;  // ✅ 向上偏移 5 cm，防止夹爪撞桌面
        // grasp_pose_out.pose.position.x += 0.00;  // ✅ 向上偏移 5 cm，防止夹爪撞桌面
        grasp_pose_out.pose.orientation.x = q_grasp.x();
        grasp_pose_out.pose.orientation.y = q_grasp.y();
        grasp_pose_out.pose.orientation.z = q_grasp.z();
        grasp_pose_out.pose.orientation.w = q_grasp.w();
        return true;
    }

    

    bool moveToPose(const geometry_msgs::PoseStamped& pose) {
        arm_group_.setStartStateToCurrentState();
        arm_group_.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return (arm_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS &&
                arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
    ros::init(argc, argv, "tiago_grasp_node_adaptive");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    GraspController controller;
    controller.initialize();

    ros::waitForShutdown();
    return 0;
}