#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_shaker_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/head_controller/command", 1);

  ros::Rate rate(50);  // 50Hz update
  double t = 0.0;

  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(1);  // 控制 1 个关节（head_1_joint）

    msg.data[0] = 0.4 * sin(t);  // 摇头：在 [-0.4, +0.4] 范围内往返
    pub.publish(msg);

    t += 0.05;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
