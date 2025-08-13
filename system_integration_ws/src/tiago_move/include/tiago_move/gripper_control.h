#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class GripperControl {
public:
    GripperControl(ros::NodeHandle& nh);

    // 打开抓取器
    void openGripper();

    // 关闭抓取器
    void closeGripper();

private:
    ros::Publisher gripper_pub_;
};

#endif // GRIPPER_CONTROL_H