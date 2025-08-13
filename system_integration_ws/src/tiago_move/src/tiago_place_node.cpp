#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class PlaceController {
public:
    PlaceController()
        : arm_group_("arm_torso"),
          tf_listener_(tf_buffer_),
          place_executed_(false)
    {}

    void initialize()
    {
        ros::NodeHandle nh("~");  // 使用私有命名空间读取参数
        arm_group_.setPlannerId("RRTstarkConfigDefault");
        arm_group_.setPoseReferenceFrame("base_footprint");
        arm_group_.setStartStateToCurrentState();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPlanningTime(5.0);

        int marker_index;
        nh.param("marker_index", marker_index, 0);
        std::string topic_name = "/marker_3d_position_" + std::to_string(marker_index);
        centroid_sub_ = nh.subscribe(topic_name, 1, &PlaceController::centroidCallback, this);

        gripper_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/parallel_gripper_controller/command", 10);

        ROS_INFO("PlaceController initialized. Subscribed to: %s", topic_name.c_str());
    }

    void centroidCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (place_executed_) return;

        geometry_msgs::PointStamped transformed_point;
        try {
            geometry_msgs::PointStamped stamped_in = *msg;
            stamped_in.header.stamp = ros::Time(0);
            tf_buffer_.transform(stamped_in, transformed_point, "base_footprint", ros::Duration(0.2));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            return;
        }

        const auto& p = transformed_point.point;
        ROS_INFO("Received place target: [%.2f, %.2f, %.2f]", p.x, p.y, p.z);

        int max_retries_ = 3;
        int retry_count_ = 0;

        while (retry_count_ < max_retries_)
        {
            ROS_INFO("Attempt #%d to place...", retry_count_ + 1);

            bool move_ok = moveAbovePlace(p);
            if (!move_ok) {
                ROS_WARN("Failed to move above place target.");
                retry_count_++;
                continue;
            }

            bool place_ok = tryPlace(p);
            if (place_ok) {
                ROS_INFO("Place succeeded!");
                ros::Duration(1.0).sleep();
                moveToInitialPose();
                place_executed_ = true;
                ros::shutdown();
                return;
            }

            ROS_WARN("Place attempt #%d failed. Retrying...", retry_count_ + 1);
            retry_count_++;
        }

        ROS_ERROR("All %d place attempts failed. Aborting.", max_retries_);
        ros::shutdown();
    }

    bool moveAbovePlace(const geometry_msgs::Point& point)
    {
        geometry_msgs::PoseStamped above_pose;
        above_pose.header.frame_id = "base_footprint";
        above_pose.header.stamp = ros::Time(0);
        above_pose.pose.position.x = point.x - 0.20;
        above_pose.pose.position.y = point.y;
        above_pose.pose.position.z = point.z + 0.20;

        tf2::Quaternion q;
        q.setRPY(M_PI/2, 0, 0);
        above_pose.pose.orientation = tf2::toMsg(q);

        return moveToPose(above_pose);
    }

    bool tryPlace(const geometry_msgs::Point& point)
    {
        geometry_msgs::PoseStamped pre_place_pose;
        pre_place_pose.header.frame_id = "base_footprint";
        pre_place_pose.header.stamp = ros::Time(0);
        pre_place_pose.pose.position.x = point.x - 0.20;
        pre_place_pose.pose.position.y = point.y;
        pre_place_pose.pose.position.z = point.z + 0.20;

        tf2::Quaternion q;
        q.setRPY(M_PI/2, 0, 0);
        pre_place_pose.pose.orientation = tf2::toMsg(q);

        if (!moveToPose(pre_place_pose)) {
            ROS_WARN("Failed to move to pre-place pose");
            return false;
        }
        ros::Duration(0.5).sleep();

        geometry_msgs::PoseStamped place_pose = pre_place_pose;
        place_pose.pose.position.x = point.x -0.20;

        if (!moveToPose(place_pose)) {
            ROS_WARN("Failed to move to place pose");
            return false;
        }

        ros::Duration(0.5).sleep();
        openGripper();
        ros::Duration(0.5).sleep();
        retreatAfterPlace(place_pose);
        // ✅ 后退 20cm
        if (!retreatAfterPlace(place_pose)) {
            ROS_WARN("Failed to retreat after placing.");
            return false;
        }

        return true;
    }

    bool retreatAfterPlace(const geometry_msgs::PoseStamped& current_pose)
    {
        geometry_msgs::PoseStamped retreat_pose = current_pose;
        retreat_pose.pose.position.x -= 0.09;
        return moveToPose(retreat_pose);
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
        point.positions = {0.15, 0.15};
        point.time_from_start = ros::Duration(1.0);
        traj.points.push_back(point);
        traj.header.stamp = ros::Time(0);
        gripper_pub_.publish(traj);
        ROS_INFO("Gripper open command published");
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
    bool place_executed_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tiago_place_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    PlaceController controller;
    controller.initialize();

    ros::waitForShutdown();
    return 0;
}
