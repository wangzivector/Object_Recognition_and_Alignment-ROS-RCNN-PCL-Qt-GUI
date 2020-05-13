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

int main(int argc, char* argv[])
{

  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  ros::init(argc, argv, "gui_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello here is gui node!");

  return a.exec();
}
