#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


class GraspController {
public:
    GraspController()
        : arm_group_("arm_torso"),
          grasp_executed_(false)
    {}

    void initialize()
    {
        arm_group_.setPlannerId("RRTConnectkConfigDefault");
        arm_group_.setPoseReferenceFrame("base_footprint");
        arm_group_.setStartStateToCurrentState();
        arm_group_.setMaxVelocityScalingFactor(1.0);
        arm_group_.setPlanningTime(5.0);

        ros::NodeHandle nh;
        centroid_sub_ = nh.subscribe("/cluster_centroid", 1, &GraspController::centroidCallback, this);
        gripper_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/parallel_gripper_controller/command", 10);

        ROS_INFO("GraspController initialized and waiting for centroid on /filtered_centroid...");
    }

    void centroidCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (grasp_executed_) return;

        ROS_INFO("Received centroid at [%.2f, %.2f, %.2f]", msg->point.x, msg->point.y, msg->point.z);
        ROS_INFO("Attempting grasp...");

        bool move_ok = moveAboveObject(msg->point);
        if (!move_ok) {
            ROS_WARN("Failed to move above object.");
            return;
        }

        openGripper();
        ros::Duration(0.5).sleep();

        moveAboveObject(msg->point);
        ros::Duration(0.5).sleep();

        bool grasp_ok = tryGrasp(msg->point);
        if (!grasp_ok) {
            ROS_WARN("Grasp attempt failed.");
            return;
        }

        ROS_INFO("Grasp succeeded!");
        grasp_executed_ = true;
        sendBaseShiftGoal();  // Move the base left after grasping the object
        ros::Duration(2.0).sleep();  // Wait a bit for base to settle

        lowerEndEffectorBy(0.1);  // Lower by 30cm
        ros::Duration(1.0).sleep(); 
        openGripper();
        ros::Duration(1.0).sleep(); 
        upperEndEffectorBy(0.1);
        ros::Duration(1.0).sleep(); 
        Rotate();
        ros::Duration(1.0).sleep(); 
        moveToHome();

       
    }

    bool moveToHome()
    {
        ROS_INFO("Moving arm to 'home' position...");

        arm_group_.setNamedTarget("home");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) && arm_group_.execute(plan)) {
            ROS_INFO("Arm successfully moved to home position.");
            return true;
        } else {
            ROS_WARN("Failed to move arm to home position.");
            return false;
        }
    }

    // Send a MoveBaseGoal to move the base 1 meter left in the robot's local frame
    void sendBaseShiftGoal()
    {
        ROS_INFO("Sending base shift goal: move left 1.0 meter.");

        // Create an ActionClient for move_base
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        // Wait for the action server to be available
        ac.waitForServer();

        // Construct the goal in the base_footprint frame
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_footprint";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 1.0;  // Move 1 meter to the left
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;  // No rotation

        // Send the goal to move_base
        ac.sendGoal(goal);

        // Wait for the result with a timeout
        bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Base move finished: %s", state.toString().c_str());
        } else {
            ROS_WARN("Base move did not finish before timeout.");
        }
    }

    void Rotate()
    {
        ROS_INFO("Sending base rotation goal: rotate 180 degrees around Z axis.");

        // Create an ActionClient for move_base
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        // Wait for the action server to be available
        ac.waitForServer();

        // Construct the goal in the base_footprint frame
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_footprint";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0.0;

        // Set rotation of 180 degrees (pi radians) around Z axis
        tf::Quaternion q;
        q.setRPY(0, 0, M_PI);  // yaw = pi
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        // Send the goal to move_base
        ac.sendGoal(goal);

        // Wait for the result with a timeout
        bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Base rotation finished: %s", state.toString().c_str());
        } else {
            ROS_WARN("Base rotation did not finish before timeout.");
        }
    }



    bool moveAboveObject(const geometry_msgs::Point& object_point)
    {
        geometry_msgs::PoseStamped approach_pose;
        approach_pose.header.frame_id = "base_footprint";
        approach_pose.header.stamp = ros::Time::now();
        approach_pose.pose.position.x = object_point.x;
        approach_pose.pose.position.y = object_point.y;
        approach_pose.pose.position.z = object_point.z + 0.50;

        tf::Quaternion q;
        q.setRPY(0, M_PI/2, 0);
        approach_pose.pose.orientation.x = q.x();
        approach_pose.pose.orientation.y = q.y();
        approach_pose.pose.orientation.z = q.z();
        approach_pose.pose.orientation.w = q.w();

        return moveToPose(approach_pose);
    }

    bool moveFarFromCenter(const geometry_msgs::Point& object_point)
    {
        geometry_msgs::PoseStamped approach_pose;
        approach_pose.header.frame_id = "base_footprint";
        approach_pose.header.stamp = ros::Time::now();
        approach_pose.pose.position.x = object_point.x+0.1;
        approach_pose.pose.position.y = object_point.y;
        approach_pose.pose.position.z = object_point.z;

        tf::Quaternion q;
        q.setRPY(0, M_PI/2, 0);
        approach_pose.pose.orientation.x = q.x();
        approach_pose.pose.orientation.y = q.y();
        approach_pose.pose.orientation.z = q.z();
        approach_pose.pose.orientation.w = q.w();

        return moveToPose(approach_pose);
    }

    bool tryGrasp(const geometry_msgs::Point& object_point)
    {
        geometry_msgs::PoseStamped pre_grasp_pose;
        pre_grasp_pose.header.frame_id = "base_footprint";
        pre_grasp_pose.header.stamp = ros::Time::now();
        pre_grasp_pose.pose.position.x = object_point.x;
        pre_grasp_pose.pose.position.y = object_point.y;
        pre_grasp_pose.pose.position.z = object_point.z + 0.2;

        tf::Quaternion q;
        q.setRPY(0, M_PI/2, 0);
        pre_grasp_pose.pose.orientation.x = q.x();
        pre_grasp_pose.pose.orientation.y = q.y();
        pre_grasp_pose.pose.orientation.z = q.z();
        pre_grasp_pose.pose.orientation.w = q.w();

        if (!moveToPose(pre_grasp_pose)) {
            ROS_WARN("Failed to move to pre-grasp pose");
            return false;
        }
        ros::Duration(2.0).sleep();

        geometry_msgs::PoseStamped grasp_pose = pre_grasp_pose;
        grasp_pose.pose.position.z = object_point.z + 0.2;
        if (!moveToPose(grasp_pose)) {
            ROS_WARN("Failed to move down to grasp pose");
            return false;
        }

        ros::Duration(2.0).sleep();
        closeGripper();
        ros::Duration(0.5).sleep();

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
        point.positions = {0.15, 0.15};
        point.time_from_start = ros::Duration(1.0);
        traj.points.push_back(point);
        traj.header.stamp = ros::Time::now();
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
        traj.header.stamp = ros::Time::now();
        gripper_pub_.publish(traj);
        ROS_INFO("Gripper close command published");
    }

 // Lower the end-effector by dz meters in Z direction (in base frame)
    bool lowerEndEffectorBy(double dz)
    {
        geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

        // Create target pose by lowering Z
        geometry_msgs::PoseStamped target_pose = current_pose;
        target_pose.pose.position.z -= dz;

        // Optionally allow small deviations in X and Y
        // (no change needed if you directly use current_pose's XY)

        arm_group_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) && arm_group_.execute(plan)) {
            ROS_INFO("Successfully lowered end-effector by %.2f meters.", dz);
            return true;
        } else {
            ROS_WARN("Failed to lower end-effector.");
            return false;
        }
    }

    bool upperEndEffectorBy(double dz)
    {
        geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

        // Create target pose by lowering Z
        geometry_msgs::PoseStamped target_pose = current_pose;
        target_pose.pose.position.z += dz;

        // Optionally allow small deviations in X and Y
        // (no change needed if you directly use current_pose's XY)

        arm_group_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm_group_.plan(plan) && arm_group_.execute(plan)) {
            ROS_INFO("Successfully lowered end-effector by %.2f meters.", dz);
            return true;
        } else {
            ROS_WARN("Failed to lower end-effector.");
            return false;
        }
    }


private:
    moveit::planning_interface::MoveGroupInterface arm_group_;
    ros::Subscriber centroid_sub_;
    ros::Publisher gripper_pub_;
    bool grasp_executed_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "filtered_grasp_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    GraspController controller;
    controller.initialize();

    ros::waitForShutdown();
    return 0;
}
