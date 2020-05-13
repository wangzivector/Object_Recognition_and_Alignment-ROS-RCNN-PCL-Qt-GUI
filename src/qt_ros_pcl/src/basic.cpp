#include <ros/ros.h>
#include <qt_ros_pcl/info.h>

void	info_Callback(const	qt_ros_pcl::info::ConstPtr	&msg_info)
{
  ROS_INFO("Basic Listener:	%s", msg_info->infomation.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic");
  ros::NodeHandle nh_sub;
  ros::Subscriber	sub_info = nh_sub.subscribe("pcl_info", 1, info_Callback);

  ROS_INFO("this node for receive final output information from gui process and pointcloud!");
  ros::spin();
  ROS_INFO("Basic node shut down! ");
}

