/*
 * gui_node.cpp
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This cppfile gui related widget visualizer
 */
#include "mainwindow.h"
#include <QApplication>
#include <gui_node.h>
#include <ros/ros.h>
#include <qt_ros_pcl/info.h>

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "gui_node");
  ros::NodeHandle nh_pub;

  qt_ros_pcl::info info_pub;
  ros::Publisher pub_info = nh_pub.advertise<qt_ros_pcl::info>("pcl_info", 0);

  ROS_INFO("Hello here is gui node!");

  int time = 5;
  ros::Rate	loop_rate(1.0);			//定义发布的频率,1HZ
  while	(ros::ok() && (time != 0))			//循环发布msg
  {
    time --;
    info_pub.infomation = "detection result.";
    pub_info.publish(info_pub);
    loop_rate.sleep();
  }

  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
