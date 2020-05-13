#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
  std::cout << "ros up !!\n";
}
