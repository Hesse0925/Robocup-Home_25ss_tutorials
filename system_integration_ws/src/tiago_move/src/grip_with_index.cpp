#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          tf_listener_(tf_buffer_),
          grasp_executed_(false)
    {}
        void initialize()
    {
        ros::NodeHandle nh("~");  // 使用私有命名空间读取参数
        arm_group_.setPlannerId("RRTstarkConfigDefault");
        arm_group_.setPoseReferenceFrame("base_footprint");
        arm_group_.setStartStateToCurrentState();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPlanningTime(5.0);

        // 获取 marker index 参数
        int marker_index;
        nh.param("marker_index", marker_index, 0);  // 默认订阅 /marker_3d_position_0
        std::string topic_name = "/marker_3d_position_" + std::to_string(marker_index);
        centroid_sub_ = nh.subscribe(topic_name, 1, &GraspController::centroidCallback, this);

        gripper_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/parallel_gripper_controller/command", 10);

        ROS_INFO("GraspController initialized. Subscribed to: %s", topic_name.c_str());
    }
    
    void centroidCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (grasp_executed_) return;
        geometry_msgs::PointStamped transformed_point;
        try {
            geometry_msgs::PointStamped stamped_in = *msg;
            stamped_in.header.stamp = ros::Time(0);  // :白色的对勾: 修复时间戳问题
            tf_buffer_.transform(stamped_in, transformed_point, "base_footprint", ros::Duration(0.2));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            return;
        }
        const auto& p = transformed_point.point;
        ROS_INFO("Received marker in base_footprint frame: [%.2f, %.2f, %.2f]", p.x, p.y, p.z);
        int max_retries_ = 5;
        int retry_count_ = 0;
        while (retry_count_ < max_retries_)
        {
            ROS_INFO("Attempt #%d to grasp...", retry_count_ + 1);
            bool move_ok = moveAboveObject(p);
            if (!move_ok) {
                ROS_WARN("Failed to move above object.");
                retry_count_++;
                continue;
            }
            openGripper();
            ros::Duration(0.5).sleep();
            bool grasp_ok = tryGrasp(p);
            if (grasp_ok) {
                ROS_INFO("Grasp succeeded!");
                ros::Duration(1.0).sleep();
                moveToInitialPose();
                grasp_executed_ = true;
                ros::shutdown();
                return;
            }
            ROS_WARN("Grasp attempt #%d failed. Retrying...", retry_count_ + 1);
            retry_count_++;
        }
        ROS_ERROR("All %d grasp attempts failed. Aborting.", max_retries_);
        ros::shutdown();
    }
    bool moveAboveObject(const geometry_msgs::Point& object_point)
    {
        geometry_msgs::PoseStamped approach_pose;
        approach_pose.header.frame_id = "base_footprint";
        approach_pose.header.stamp = ros::Time(0);
        approach_pose.pose.position.x = object_point.x-0.30;
        approach_pose.pose.position.y = object_point.y;
        approach_pose.pose.position.z = object_point.z + 0.10;
        tf2::Quaternion q;
        q.setRPY(M_PI/2, 0, 0);  // z 向下抓
        approach_pose.pose.orientation = tf2::toMsg(q);
        return moveToPose(approach_pose);
    }
    bool tryGrasp(const geometry_msgs::Point& object_point)
    {
        geometry_msgs::PoseStamped pre_grasp_pose;
        pre_grasp_pose.header.frame_id = "base_footprint";
        pre_grasp_pose.header.stamp = ros::Time(0);
        pre_grasp_pose.pose.position.x = object_point.x - 0.20;
        pre_grasp_pose.pose.position.y = object_point.y;
        pre_grasp_pose.pose.position.z = object_point.z + 0.10;
        tf2::Quaternion q;
        q.setRPY(M_PI/2, 0, 0);  // z 向下抓
        pre_grasp_pose.pose.orientation = tf2::toMsg(q);
        if (!moveToPose(pre_grasp_pose)) {
            ROS_WARN("Failed to move to pre-grasp pose");
            return false;
        }
        ros::Duration(2.0).sleep();
        geometry_msgs::PoseStamped grasp_pose = pre_grasp_pose;
        grasp_pose.pose.position.x = object_point.x - 0.20;
        if (!moveToPose(grasp_pose)) {
            ROS_WARN("Failed to move down to grasp pose");
            return false;
        }
        ros::Duration(0.5).sleep();
        closeGripper();
        ros::Duration(2.0).sleep();
        return moveAboveObject(object_point);
    }
    bool moveToPose(const geometry_msgs::PoseStamped& pose)
    {
        arm_group_.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return arm_group_.plan(plan) && arm_group_.execute(plan);
    }
    void openGripper()
    {
        trajectory_msgs::JointTrajectory traj;
        traj.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.15, 0.15};  // 可根据你的 gripper 调整开合角度
        point.time_from_start = ros::Duration(1.0);
        traj.points.push_back(point);
        traj.header.stamp = ros::Time(0);
        gripper_pub_.publish(traj);
        ROS_INFO("Gripper open command published");
    }
    void closeGripper()
    {
        trajectory_msgs::JointTrajectory traj;
        traj.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0};
        point.time_from_start = ros::Duration(1.0);
        traj.points.push_back(point);
        traj.header.stamp = ros::Time(0);
        gripper_pub_.publish(traj);
        ROS_INFO("Gripper close command published");
    }
    void moveToInitialPose()
    {
        std::map<std::string, double> joint_positions;
        joint_positions["torso_lift_joint"] = 0.150;
        joint_positions["arm_1_joint"] = 0.200;
        joint_positions["arm_2_joint"] = -1.339;
        joint_positions["arm_3_joint"] = -0.200;
        joint_positions["arm_4_joint"] = 1.938;
        joint_positions["arm_5_joint"] = -1.570;
        joint_positions["arm_6_joint"] = 0.000;
        joint_positions["arm_7_joint"] = 0.000;
        arm_group_.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) && arm_group_.execute(plan)) {
            ROS_INFO("Returned to initial joint pose.");
        } else {
            ROS_WARN("Failed to move to initial joint pose.");
        }
    }
private:
    moveit::planning_interface::MoveGroupInterface arm_group_;
    ros::Subscriber centroid_sub_;
    ros::Publisher gripper_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool grasp_executed_;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tiago_grasp_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    GraspController controller;
    controller.initialize();
    ros::waitForShutdown();
    return 0;
}








