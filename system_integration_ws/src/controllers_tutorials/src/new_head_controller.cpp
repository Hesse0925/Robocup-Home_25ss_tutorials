#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

namespace controllers_tutorials
{

class NewHeadController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  NewHeadController() {}
  ~NewHeadController() override {}

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) override
  {
    ROS_INFO("[NewHeadController] init() called");

    if (!nh.getParam("joints", joint_names_)) {
      ROS_ERROR("[NewHeadController] Could not find 'joints' parameter on the parameter server.");
      return false;
    }

    if (joint_names_.empty()) {
      ROS_ERROR("[NewHeadController] 'joints' parameter is empty.");
      return false;
    }

    for (const std::string& joint_name : joint_names_) {
      try {
        hardware_interface::JointHandle handle = hw->getHandle(joint_name);
        joint_handles_.push_back(handle);
        ROS_INFO_STREAM("[NewHeadController] Found joint handle: " << joint_name);
      } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("[NewHeadController] Failed to get joint handle for " << joint_name << ": " << e.what());
        return false;
      }
    }

    start_time_ = ros::Time::now();
    ROS_INFO("[NewHeadController] Initialization successful");
    return true;
  }

  void starting(const ros::Time& time) override
  {
    start_time_ = time;
    ROS_INFO("[NewHeadController] Controller starting...");
  }

  void update(const ros::Time& time, const ros::Duration& period) override
  {
    double elapsed = (time - start_time_).toSec();

    for (size_t i = 0; i < joint_handles_.size(); ++i) {
      double cmd = 0.0;
      if (i == 0) {
        cmd = 0.5 * sin(elapsed);         // head_1_joint: 左右
      } else if (i == 1) {
        cmd = 0.25 * cos(2 * elapsed);    // head_2_joint: 上下（幅度小，频率高）
      }
      joint_handles_[i].setCommand(cmd);
    }
  }


  void stopping(const ros::Time& time) override
  {
    ROS_INFO("[NewHeadController] Controller stopping.");
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  ros::Time start_time_;
};

} // namespace controllers_tutorials

PLUGINLIB_EXPORT_CLASS(controllers_tutorials::NewHeadController, controller_interface::ControllerBase)
