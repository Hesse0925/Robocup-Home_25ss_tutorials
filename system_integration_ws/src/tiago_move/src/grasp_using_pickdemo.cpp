#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <vector>
#include <grasp_generator/GenerateGrasps.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          grasp_executed_(false),
          marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("grasp_markers", 1)),
          gripper_pub_(nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10)) {}

    void initialize() {
        arm_group_.setPlannerId("RRTConnectkConfigDefault");
        arm_group_.setPoseReferenceFrame("base_footprint");
        arm_group_.setStartStateToCurrentState();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPlanningTime(5.0);

        pose_sub_ = nh_.subscribe("/object_pose", 1, &GraspController::poseCallback, this);

        ROS_INFO("GraspController initialized.");
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (grasp_executed_) return;

        geometry_msgs::PoseStamped object_pose = *msg;

        ROS_INFO("Received object pose at [%.2f, %.2f, %.2f]",
                 object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

        // 生成抓取姿态
        std::vector<geometry_msgs::PoseStamped> grasp_poses = generateGraspPoses(object_pose);

        // 可视化抓取姿态
        visualizeGraspPoses(grasp_poses);

        // 执行抓取
        if (executeGrasp(grasp_poses)) {
            ROS_INFO("Grasp succeeded!");
            grasp_executed_ = true;
        } else {
            ROS_WARN("Grasp failed.");
        }
    }

    std::vector<geometry_msgs::PoseStamped> generateGraspPoses(const geometry_msgs::PoseStamped& object_pose) {
        std::vector<geometry_msgs::PoseStamped> grasp_poses;

        // 创建 service client（使用 this->nh_）
        ros::ServiceClient client = nh_.serviceClient<grasp_generator::GenerateGrasps>("/generate_grasps");

        grasp_generator::GenerateGrasps srv;
        srv.request.object_pose = object_pose;

        ROS_INFO("Calling grasp_generator service...");

        if (client.call(srv)) {
            ROS_INFO("Grasp generator service call successful. Got %lu grasps.", srv.response.grasps.size());
            for (const auto& grasp : srv.response.grasps) {
                grasp_poses.push_back(grasp.grasp_pose);
            }
        } else {
            ROS_ERROR("Failed to call /generate_grasps service.");
        }

        return grasp_poses;
    }

    void visualizeGraspPoses(const std::vector<geometry_msgs::PoseStamped>& grasp_poses) {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < grasp_poses.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = grasp_poses[i].header.frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "grasp_markers";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose = grasp_poses[i].pose;
            marker.scale.x = 0.1;  // 箭头长度
            marker.scale.y = 0.02; // 箭头宽度
            marker.scale.z = 0.02; // 箭头高度

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        marker_pub_.publish(marker_array);
        ROS_INFO("Published %lu grasp markers visual.", grasp_poses.size());
    }

    bool executeGrasp(const std::vector<geometry_msgs::PoseStamped>& grasp_poses) {
        for (const auto& grasp_pose : grasp_poses) {
            // 移动到抓取前的位置
            geometry_msgs::PoseStamped pre_grasp_pose = grasp_pose;
            pre_grasp_pose.pose.position.z += 0.1;  // 提高以避免碰撞

            if (!moveToPose(pre_grasp_pose)) {
                ROS_WARN("Failed to move to pre-grasp pose.");
                continue;
            }

            // 移动到抓取位置
            if (!moveToPose(grasp_pose)) {
                ROS_WARN("Failed to move to grasp pose.");
                continue;
            }

            // 关闭抓取器
            closeGripper();
            ros::Duration(1.0).sleep();  // 模拟抓取时间

            // 抓取成功
            return true;
        }

        return false;  // 所有抓取姿态都失败
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
        trajectory_msgs::JointTrajectory msg;
        msg.joint_names.push_back("gripper_finger_joint");
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0.04);  // 打开抓取器的目标位置
        point.time_from_start = ros::Duration(1.0);
        msg.points.push_back(point);

        gripper_pub_.publish(msg);
        ROS_INFO("Gripper opened.");
    }

    void closeGripper() {
        trajectory_msgs::JointTrajectory msg;
        msg.joint_names.push_back("gripper_finger_joint");
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0.0);  // 关闭抓取器的目标位置
        point.time_from_start = ros::Duration(1.0);
        msg.points.push_back(point);

        gripper_pub_.publish(msg);
        ROS_INFO("Gripper closed.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher gripper_pub_;  // 抓取器控制发布器
    moveit::planning_interface::MoveGroupInterface arm_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
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
