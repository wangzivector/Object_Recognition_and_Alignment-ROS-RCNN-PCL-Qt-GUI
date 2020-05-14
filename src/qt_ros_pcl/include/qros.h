/*
 * qros.h
 *
 *  Created on: May 13, 2020
 *      Author: wangzivector
 * This hfile handle ros related stuff
 */
#ifndef QROS_H
#define QROS_H
#include <QString>
#include <qt_ros_pcl/info.h>
#include <ros/ros.h>

class qros
{
public:
  qros();
  ~qros();
  bool qrosPublish(QString info);

private:
  /// ros stuff that should create after ros::init
  ros::NodeHandle nh_pub;

  /// also this should maintain when publish msg occurs.
  ros::Publisher pub_info = nh_pub.advertise<qt_ros_pcl::info>("pcl_info", 0);
  ros::Publisher* pub_infox;
  qt_ros_pcl::info info_pub;
};

#endif // QROS_H
