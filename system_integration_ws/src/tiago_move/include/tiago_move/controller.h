#ifndef TIAGO_MOVE_H_
#define TIAGO_MOVE_H_

#include <string>
#include <vector>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

namespace tiago_move
{

  class Controller
  {
    private:
      moveit::planning_interface::MoveGroupInterface body_planner_{"arm_torso"};

    public:

      Controller();

      virtual ~Controller();

      bool initialize(ros::NodeHandle& nh);

      move_base_msgs::MoveBaseGoal createGoal(std::vector<double>&);

      int move_arm(std::vector<double>&);

      // === 已有成员 ===
      std::vector<double> target_pose;
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac{"move_base", true};
      std::vector<move_base_msgs::MoveBaseGoal> nav_goals;

      // === 新增：面向桌子和低头的位姿目标 ===
      std::vector<double> face_table_pose;
      std::vector<double> look_down_pose;
  };
}

#endif
